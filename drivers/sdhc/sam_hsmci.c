/*
 * Copyright 2022 Nikhef
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT atmel_sam_hsmci


/*
	Note: This driver for now only works with PDC mode! 

	Families which should work:
		SAM4E, SAM4S

	Families which require XDMA and do not work (yet):
		SAMS, SAMV, SAME
		
 */


#include <zephyr/drivers/sdhc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <soc.h>

#ifndef HSMCI_MR_PDCMODE
#error "Sorry, only SAM4E/SAM4S are supported at the moment"
#endif


LOG_MODULE_REGISTER(hsmci, CONFIG_SDHC_LOG_LEVEL);

#define _HSMCI_DEFAULT_TIMEOUT 5000
#define _HSMCI_MAX_FREQ (SOC_ATMEL_SAM_MCK_FREQ_HZ >> 4)
#define _HSMCI_MIN_FREQ (_HSMCI_MAX_FREQ / 0x200)		// only valid when MCI has ODD bit

#define _HSMCI_SR_ERR 	(HSMCI_SR_RINDE | HSMCI_SR_RDIRE | HSMCI_SR_RCRCE | \
						 HSMCI_SR_RENDE | HSMCI_SR_RTOE  | HSMCI_SR_DCRCE | \
						 HSMCI_SR_DTOE  | HSMCI_SR_CSTOE | HSMCI_SR_OVRE  | \
						 HSMCI_SR_UNRE )


static const uint8_t _resp2size[] = {
	[SD_RSP_TYPE_NONE] = HSMCI_CMDR_RSPTYP_NORESP,
	[SD_RSP_TYPE_R1] = HSMCI_CMDR_RSPTYP_48_BIT,
	[SD_RSP_TYPE_R1b] = HSMCI_CMDR_RSPTYP_R1B,
	[SD_RSP_TYPE_R2] = HSMCI_CMDR_RSPTYP_136_BIT,
	[SD_RSP_TYPE_R3] = HSMCI_CMDR_RSPTYP_48_BIT,
	[SD_RSP_TYPE_R4] = HSMCI_CMDR_RSPTYP_48_BIT,
	[SD_RSP_TYPE_R5] = 0 /*?*/,
	[SD_RSP_TYPE_R5b] = 0 /*?*/,
	[SD_RSP_TYPE_R6] = HSMCI_CMDR_RSPTYP_48_BIT,
	[SD_RSP_TYPE_R7] = HSMCI_CMDR_RSPTYP_48_BIT,
};

// timeout multiplier shift (actual value is 1 << _mul_shift[*])
static const uint8_t _mul_shift[] = { 0, 4, 7, 8, 10, 12, 16, 20 };
static const uint8_t _mul_shift_size = 8;

struct sam_hsmci_config 
{
	Hsmci *base;
	uint8_t periph_id;
	const struct pinctrl_dev_config *pincfg;
	struct gpio_dt_spec carrier_detect;
};

struct sam_hsmci_data 
{
	bool open_drain;
	uint8_t cmd_in_progress;
	struct k_mutex mtx;
};


static int sam_hsmci_reset(const struct device *dev)
{
	LOG_DBG("sam_hsmci_reset()");
	// TODO make it a better reset!
	const struct sam_hsmci_config *config = dev->config;
	config->base->HSMCI_CR = HSMCI_CR_SWRST;
	return 0;
}


static int sam_hsmci_get_host_props(const struct device *dev,
	struct sdhc_host_props *props)
{
	LOG_DBG("sam_hsmci_get_host_props()");
	memset(props, 0, sizeof(*props));
	props->f_max = _HSMCI_MAX_FREQ;
	props->f_min = _HSMCI_MIN_FREQ;
	// true not working yet
	props->host_caps.high_spd_support = false;
	props->power_delay = 500;
	props->is_spi = false;
	props->max_current_330 = 4;

	return 0;	
}


static int sam_hsmci_set_io(const struct device *dev, struct sdhc_io *ios)
{
	LOG_DBG("sam_hsmci_set_io(clock=%d, bus_width=%d, timing=%d, mode=%d)", ios->clock, ios->bus_width, ios->timing, ios->bus_mode);

	const struct sam_hsmci_config *config = dev->config;
	struct sam_hsmci_data *data = dev->data;
	Hsmci * hsmci = config->base; 

	if (ios->clock > 0)
	{
		int div_val = SOC_ATMEL_SAM_MCK_FREQ_HZ / ios->clock - 2;
		if (div_val < 0) div_val = 0;
		if (div_val > 0x1ff) div_val = 0x1ff;

		LOG_DBG("divider: %d (freq=%d)", div_val, _HSMCI_MAX_FREQ);

		hsmci->HSMCI_MR &= ~HSMCI_MR_CLKDIV_Msk;
		hsmci->HSMCI_MR |= ( ( div_val & 1 ) ? HSMCI_MR_CLKODD : 0 ) |
				HSMCI_MR_CLKDIV(div_val >> 1);
	}
	// Note: HSMCI buswidth of 8 may be supported by some ATMEL devices. 
	//       maybe we can auto-detect this?
	hsmci->HSMCI_SDCR &= ~HSMCI_SDCR_SDCBUS_Msk;

	switch (ios->bus_width)
	{
	case SDHC_BUS_WIDTH1BIT:
		hsmci->HSMCI_SDCR = HSMCI_SDCR_SDCBUS_1;
		break;
	case SDHC_BUS_WIDTH4BIT:
		hsmci->HSMCI_SDCR = HSMCI_SDCR_SDCBUS_4;
		break;
	case SDHC_BUS_WIDTH8BIT:
		// I have yet to encounter an ATMEL SAM which supports 8 lane
		// So, for now we'll just say NO.
		return -ENOTSUP;
	default:
		return -ENOTSUP;
	}

	// open-drain is part of the command consutrction algorithm
	data->open_drain = ( ios->bus_mode == SDHC_BUSMODE_OPENDRAIN );

	switch (ios->timing)
	{
	case SDHC_TIMING_LEGACY:
		hsmci->HSMCI_CFG &= ~HSMCI_CFG_HSMODE;
		break;
	case SDHC_TIMING_HS:
		hsmci->HSMCI_CFG |= HSMCI_CFG_HSMODE;
		break;
	default:
		return -ENOTSUP;
	}


	return 0;
}


static int sam_hsmci_init(const struct device *dev)
{
	const struct sam_hsmci_config *config = dev->config;

	/* Connect pins to the peripheral */
	int ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0)
	{
		LOG_ERR("pinctrl_apply_state() => %d", ret);
		return ret;
	}
	/* Enable module's clock */
	soc_pmc_peripheral_enable(config->periph_id);	

	// init carrier detect (if set)
	if (config->carrier_detect.port != NULL) 
	{
		if (!device_is_ready(config->carrier_detect.port)) 
		{
			LOG_ERR("GPIO port for carrier-detect pin is not ready");
			return -ENODEV;
		}
		ret = gpio_pin_configure_dt(&config->carrier_detect, GPIO_INPUT);
		if (ret < 0) 
		{
			LOG_ERR("Couldn't configure carrier-detect pin; (%d)", ret);
			return ret;
		}
	}	

	Hsmci * hsmci = config->base; 
	// reset the device
	hsmci->HSMCI_CR |= HSMCI_CR_SWRST;
	// Enable the HSMCI multi-media interface
	hsmci->HSMCI_CR |= HSMCI_CR_MCIEN;
	// enable read/write proof, we don't want to loose data
	hsmci->HSMCI_MR = HSMCI_MR_RDPROOF | HSMCI_MR_WRPROOF;

	return 0;
}


static int sam_hsmci_get_card_present(const struct device *dev)
{
	LOG_DBG("sam_hsmci_get_card_present()");
	const struct sam_hsmci_config *config = dev->config;

	// no carrier detect, assume it is present
	if (config->carrier_detect.port == NULL) 
	{
		return 1;		
	}

	return gpio_pin_get_dt(&config->carrier_detect);
}


static int sam_hsmci_card_busy(const struct device *dev)
{
	LOG_DBG("sam_hsmci_card_busy()");
	const struct sam_hsmci_config *config = dev->config;
	Hsmci * hsmci = config->base; 

	return (hsmci->HSMCI_SR & HSMCI_SR_NOTBUSY) == 0;
}


static void sam_hsmci_send_clocks(Hsmci * hsmci)
{
	// Configure command
	hsmci->HSMCI_MR &= ~(HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF | HSMCI_MR_FBYTE);
	// Write argument
	hsmci->HSMCI_ARGR = 0;
	// Write and start initialization command
	hsmci->HSMCI_CMDR = HSMCI_CMDR_RSPTYP_NORESP
			| HSMCI_CMDR_SPCMD_INIT
			| HSMCI_CMDR_OPDCMD_OPENDRAIN;
	// Wait end of initialization command
	while (!(hsmci->HSMCI_SR & HSMCI_SR_CMDRDY));
	hsmci->HSMCI_MR |= HSMCI_MR_WRPROOF | HSMCI_MR_RDPROOF;
}


static int sam_hsmci_send_cmd(Hsmci * hsmci, struct sdhc_command *cmd, uint32_t cmdr,	struct sam_hsmci_data *data)
{
	uint32_t sr;
	hsmci->HSMCI_ARGR = cmd->arg;

	// build SD command
	cmdr |= HSMCI_CMDR_CMDNB(cmd->opcode);
	if (data->open_drain) 
	{
		cmdr |= HSMCI_CMDR_OPDCMD_OPENDRAIN;
	}

	// we'll just take 64 clocks for max latency
	cmdr |= HSMCI_CMDR_MAXLAT_64;
	// get native response type
	uint8_t nrt = cmd->response_type & SDHC_NATIVE_RESPONSE_MASK; 
	// R7 is the highest we support
	if (nrt > SD_RSP_TYPE_R7) return -ENOTSUP;
	// map response to size
	cmdr |= _resp2size[nrt];
	// issue command
	hsmci->HSMCI_CMDR = cmdr;	
	do
	{
		sr = hsmci->HSMCI_SR;
		// special case ,ignore CRC status if response is R3 to clear it
		if (nrt == SD_RSP_TYPE_R3 || nrt == SD_RSP_TYPE_NONE)
		{
			sr &= ~HSMCI_SR_RCRCE;
		}
		// Respond to any error
		if ((sr & _HSMCI_SR_ERR) != 0) 
		{
			LOG_DBG("Status register error bits: %08x", sr & _HSMCI_SR_ERR);
			return -EIO;
		}
	} while (!(sr & HSMCI_SR_CMDRDY));

	if (nrt == SD_RSP_TYPE_R1b) 
	{
		do 
		{
			sr = hsmci->HSMCI_SR;
		} while (!((sr & HSMCI_SR_NOTBUSY) && ((sr & HSMCI_SR_DTIP) == 0)));
	}

	// RSPR is just a FIFO
	cmd->response[3] = hsmci->HSMCI_RSPR[0];
	cmd->response[2] = hsmci->HSMCI_RSPR[0];
	cmd->response[1] = hsmci->HSMCI_RSPR[0];
	cmd->response[0] = hsmci->HSMCI_RSPR[0];
	return 0;
}


static int sam_hsmci_wait_write_end(Hsmci * hsmci)
{
	uint32_t sr = 0;

	// Wait end of transfer
	// Note: no need of timeout, because it is include in HSMCI, see DTOE bit.
	do {
		sr = hsmci->HSMCI_SR;
		if (sr &
				(HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			LOG_DBG("PDC sr 0x%08x error", sr);
			return -EIO;
		}
	} while (!(sr & HSMCI_SR_TXBUFE));


	// if (hsmci_transfert_pos < ((uint32_t)hsmci_block_size * hsmci_nb_block)) {
	// 	return 0;
	// }
	// It is the last transfer, then wait command completed
	// Note: no need of timeout, because it is include in HSMCI, see DTOE bit.
	do {
		sr = hsmci->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			LOG_DBG("PDC sr 0x%08x last transfer error", sr);
			return -EIO;
		}
	} while (!(sr & HSMCI_SR_NOTBUSY));

	if (!(hsmci->HSMCI_SR & HSMCI_SR_FIFOEMPTY)) return -EIO;
	return 0;
}


static int sam_hsmci_wait_read_end(Hsmci * hsmci)
{
	uint32_t sr;
	do 
	{
		sr = hsmci->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			LOG_DBG("PDC sr 0x%08x error", sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE));
			return -EIO;
		}
	} while (!(sr & HSMCI_SR_RXBUFF));

	do 
	{
		sr = hsmci->HSMCI_SR;
		if (sr & (HSMCI_SR_UNRE | HSMCI_SR_OVRE | \
				HSMCI_SR_DTOE | HSMCI_SR_DCRCE)) {
			return -EIO;
		}
	} while (!(sr & HSMCI_SR_XFRDONE));
	return 0;
}


static int sam_hsmci_write_timeout(Hsmci * hsmci, int timeout_ms)
{
	LOG_DBG("sam_hsmci_write_timeout(timeout_ms=%d)", timeout_ms);
	// convert to clocks (coarsely)
	int clocks = ATMEL_SAM_DT_CPU_CLK_FREQ_HZ / 1000 * timeout_ms;
	
	for (int i = 0; i < _mul_shift_size; i++)
	{
		int mul = 1 << _mul_shift[i];
		int max_clock = 15 * mul;
		if (max_clock > clocks) 
		{
			hsmci->HSMCI_DTOR = ( ( i << HSMCI_DTOR_DTOMUL_Pos ) & HSMCI_DTOR_DTOMUL_Msk ) |
				HSMCI_DTOR_DTOCYC ( (  clocks + mul - 1 ) / mul );
			return 0;
		}
	}
	// So, if it is > maximum timeout... we'll just put it on the maximum the driver supports
	// its not nice.. but it should work.. what else is there to do?
	hsmci->HSMCI_DTOR = HSMCI_DTOR_DTOMUL_Msk | HSMCI_DTOR_DTOCYC_Msk;
	return 0;
}


static int sam_hsmci_request_inner(const struct device *dev,
			struct sdhc_command *cmd,
			struct sdhc_data *sdhc_data)
{
	LOG_DBG("sam_hsmci_request(opcode=%d, arg=%08x, data=%08x, rsptype=%d)", cmd->opcode, cmd->arg, (uint32_t)sdhc_data, cmd->response_type & SDHC_NATIVE_RESPONSE_MASK);
	const struct sam_hsmci_config *config = dev->config;
	struct sam_hsmci_data *data = dev->data;
	Hsmci * hsmci = config->base; 
	uint32_t sr = 0;
	int ret = 0;

	// when CMD0 , we'll prefix 74 clocks
	if (cmd->opcode == SD_GO_IDLE_STATE) 
	{
		sam_hsmci_send_clocks(hsmci);
	}

	uint32_t cmdr = 0;
	bool is_write = false;
	// now we'll need to setup data-transfer, if any
	if (sdhc_data)
	{
		// data transfer required.
		cmdr |= HSMCI_CMDR_TRCMD_START_DATA;

		ret = sam_hsmci_write_timeout(hsmci, cmd->timeout_ms);
		if (ret != 0) return ret;

		switch (cmd->opcode) {
		case SD_WRITE_SINGLE_BLOCK:
			cmdr |= HSMCI_CMDR_TRTYP_SINGLE;
			cmdr |= HSMCI_CMDR_TRDIR_WRITE;
			is_write = true;
			break;			
		case SD_WRITE_MULTIPLE_BLOCK:
			is_write = true;
			cmdr |= HSMCI_CMDR_TRTYP_MULTIPLE;
			cmdr |= HSMCI_CMDR_TRDIR_WRITE;
			break;
		case SD_APP_SEND_SCR:
		case SD_SWITCH:
		case SD_READ_SINGLE_BLOCK:
			cmdr |= HSMCI_CMDR_TRTYP_SINGLE;
			cmdr |= HSMCI_CMDR_TRDIR_READ;
			break;
		case SD_READ_MULTIPLE_BLOCK:
			cmdr |= HSMCI_CMDR_TRTYP_MULTIPLE;
			cmdr |= HSMCI_CMDR_TRDIR_READ;
			break;
		case SD_APP_SEND_NUM_WRITTEN_BLK:
			break;
		default:
			return -ENOTSUP;
		}
		
		uint32_t size;
		if ((sdhc_data->block_size & 0x3) == 0 && (((uint32_t)sdhc_data->data) & 0x3) == 0)
		{
			size = (sdhc_data->block_size + 3) >> 2;
			hsmci->HSMCI_MR &= ~HSMCI_MR_FBYTE;
		} else {
			// we can do byte mode
			size = sdhc_data->block_size;
			hsmci->HSMCI_MR |= HSMCI_MR_FBYTE;
		}
		
		hsmci->HSMCI_MR |= HSMCI_MR_PDCMODE;
		// we can do word mode
		hsmci->HSMCI_BLKR = 
			HSMCI_BLKR_BLKLEN(sdhc_data->block_size) |
			HSMCI_BLKR_BCNT(sdhc_data->blocks);

		hsmci->HSMCI_RNCR = 0;		

		if (is_write) {
			hsmci->HSMCI_TCR = size;
			hsmci->HSMCI_TPR = (uint32_t) sdhc_data->data;
		} else {
			hsmci->HSMCI_RCR = size;
			hsmci->HSMCI_RPR = (uint32_t) sdhc_data->data;
			hsmci->HSMCI_PTCR = HSMCI_PTCR_RXTEN;
		}
	} else {
		hsmci->HSMCI_MR &= ~HSMCI_MR_PDCMODE;
	}

	ret = sam_hsmci_send_cmd(hsmci, cmd, cmdr, data);

	if (sdhc_data && ret == 0)
	{
		if (is_write) 
		{
			hsmci->HSMCI_PTCR = HSMCI_PTCR_TXTEN;
			ret = sam_hsmci_wait_write_end(hsmci);
			hsmci->HSMCI_PTCR = HSMCI_PTCR_TXTDIS;
		} else {
			ret = sam_hsmci_wait_read_end(hsmci);
			hsmci->HSMCI_PTCR = HSMCI_PTCR_RXTDIS;
		}
	}

	hsmci->HSMCI_MR &= ~HSMCI_MR_PDCMODE;

	sr = hsmci->HSMCI_SR; 
	LOG_DBG("RSP0=%08x, RPS1=%08x, RPS2=%08x,RSP3=%08x, SR=%08x",
		cmd->response[0], cmd->response[1], cmd->response[2], cmd->response[3], sr);
	if (sr & HSMCI_SR_RXRDY) 
	{
		LOG_WRN("Current command did not empty RX buffer! Bad command!");
	}
	return ret;
}


static void sam_hsmci_abort(const struct device *dev)
{
	const struct sam_hsmci_config *config = dev->config;
	Hsmci * hsmci = config->base; 
	hsmci->HSMCI_PTCR = HSMCI_PTCR_RXTDIS | HSMCI_PTCR_TXTDIS;
	struct sdhc_command cmd = 
	{
		.opcode = SD_STOP_TRANSMISSION,
		.arg = 0,
		.response_type = SD_RSP_TYPE_NONE
	};
	sam_hsmci_request_inner(dev, &cmd, NULL);			
}


static int sam_hsmci_request(const struct device *dev,
			struct sdhc_command *cmd,
			struct sdhc_data *sdhc_data)
{
	int ret = 0;
	struct sam_hsmci_data *dev_data = dev->data;
	int busy_timeout = _HSMCI_DEFAULT_TIMEOUT;

	// Thanks Daniel DeGrasse for this nice outer-loop
	ret = k_mutex_lock(&dev_data->mtx, K_MSEC(cmd->timeout_ms));
	if (ret) {
		LOG_ERR("Could not access card");
		return -EBUSY;
	}
	do {	
		ret = sam_hsmci_request_inner(dev, cmd, sdhc_data);
		if (sdhc_data && ret) 
		{
			sam_hsmci_abort(dev);
			while (busy_timeout > 0) 
			{
				if (!sam_hsmci_card_busy(dev)) {
					break;
				}
				/* Wait 125us before polling again */
				k_busy_wait(125);
				busy_timeout -= 125;
			}
			if (busy_timeout <= 0) 
			{
				LOG_ERR("Card did not idle after CMD12");
				k_mutex_unlock(&dev_data->mtx);
				return -ETIMEDOUT;
			}
		}
	} while (ret != 0 && (cmd->retries-- > 0));
	k_mutex_unlock(&dev_data->mtx);
	
	

	return ret;
}


static const struct sdhc_driver_api hsmci_api = {
	.reset = sam_hsmci_reset,
	.get_host_props = sam_hsmci_get_host_props,
	.set_io = sam_hsmci_set_io,
	.get_card_present = sam_hsmci_get_card_present,
	.request = sam_hsmci_request,
	.card_busy = sam_hsmci_card_busy,
};


#define SAM_HSMCI_INIT(N)							                \
										                            \
	PINCTRL_DT_INST_DEFINE(N);						                \
										                            \
	static const struct sam_hsmci_config hsmci_##N##_config = {		\
		.base = (Hsmci *) DT_INST_REG_ADDR(N),			            \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(N),			    \
		.periph_id =  DT_INST_PROP(N, peripheral_id),	            \
		.carrier_detect = GPIO_DT_SPEC_INST_GET_OR(N, cd_gpios, {0})\
	};									                            \
										                            \
																	\
	static struct sam_hsmci_data hsmci_##N##_data = {				\
	};																\
										                            \
	DEVICE_DT_INST_DEFINE(N,						                \
		&sam_hsmci_init,						                    \
		NULL,								                        \
		&hsmci_##N##_data,						                    \
		&hsmci_##N##_config,						                \
		POST_KERNEL,							                    \
		CONFIG_SDHC_INIT_PRIORITY,					                \
		&hsmci_api);

DT_INST_FOREACH_STATUS_OKAY(SAM_HSMCI_INIT)


