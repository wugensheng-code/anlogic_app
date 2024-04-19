/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_dw_apb_uart

/**
 * @brief Driver for UART port on STM32 family processor.
 * @note  LPUART and U(S)ART have the same base and
 *        majority of operations are performed the same way.
 *        Please validate for newly added series.
 */

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/init.h>

#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <al_uart_ll.h>
#include <al_uart_dev.h>
LOG_MODULE_REGISTER(uart_dw_apb, CONFIG_UART_LOG_LEVEL);

#define DW_APB_UART_DEFAULT_BAUDRATE	115200
#define DW_APB_UART_DEFAULT_PARITY	UART_CFG_PARITY_NONE
#define DW_APB_UART_DEFAULT_STOP_BITS	UART_CFG_STOP_BITS_1
#define DW_APB_UART_DEFAULT_DATA_BITS	UART_CFG_DATA_BITS_8

struct uart_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t user_cb;
	void *user_data;
#endif

	const AL_UINTPTR uart;
	uint32_t clock;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API) || \
	defined(CONFIG_PM)
	uart_irq_config_func_t irq_config_func;
#endif
};

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || \
	defined(CONFIG_UART_ASYNC_API) || \
	defined(CONFIG_PM)

static void uart_dw_isr(const struct device *dev)
{
	struct uart_data *data = dev->data;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	if (data->user_cb) {
		data->user_cb(dev, data->user_data);
	}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API || CONFIG_PM */

static int uart_dw_poll_in(const struct device *dev, unsigned char *c)
{
	struct uart_data *data = dev->data;
	int ret = -1;

	if ((AlUart_ll_IsRxDataReady(data->uart))) {
		*c = AlUart_ll_RecvByte(data->uart);
		ret = 0;
	}

	return ret;

}

static void uart_dw_poll_out(const struct device *dev, unsigned char c)
{
  struct uart_data *data = dev->data;
  unsigned int key;

  AlUart_ll_EnableThreIntr(data->uart, false);

  while(1) {
    if (AlUart_ll_GetLsrThreState(data->uart)) {
      AlUart_ll_SendByte(data->uart, c);
	  break;
    }
  }

  while (!(AlUart_ll_IsEmptyTsrAndTxFifo(data->uart)));

}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_dw_irq_tx_enable(const struct device *dev)
{
	struct uart_data *data = dev->data;

    AlUart_ll_EnableThreIntr(data->uart, AL_TRUE);
    AlUart_ll_SetTxIntr(data->uart, AL_FUNC_ENABLE);
}

static void uart_dw_irq_tx_disable(const struct device *dev)
{
	struct uart_data *data = dev->data;

    AlUart_ll_SetTxIntr(data->uart, AL_FUNC_DISABLE);
}

static int uart_dw_irq_tx_ready(const struct device *dev)
{
	struct uart_data *data = dev->data;

	while (!(AlUart_ll_GetThreState(data->uart)) && (AlUart_ll_IsFifosEnable(data->uart)));
	return !AlUart_ll_GetLsrThreState(data->uart);
}

static int uart_dw_irq_tx_complete(const struct device *dev)
{
	struct uart_data *data = dev->data;

    while (!(AlUart_ll_GetThreState(data->uart)) && (AlUart_ll_IsFifosEnable(data->uart)));
    return !AlUart_ll_GetLsrThreState(data->uart);
}

static void uart_dw_irq_rx_enable(const struct device *dev)
{
	struct uart_data *data = dev->data;

    AlUart_ll_EnableThreIntr(data->uart, AL_TRUE);
    AlUart_ll_SetRxIntr(data->uart, AL_FUNC_ENABLE);
}

static void uart_dw_irq_rx_disable(const struct device *dev)
{
	struct uart_data *data = dev->data;

    AlUart_ll_SetRxIntr(data->uart, AL_FUNC_DISABLE);
}

static int uart_dw_irq_rx_ready(const struct device *dev)
{
	struct uart_data *data = dev->data;

	return AlUart_ll_IsRxDataReady(data->uart);
}

static void uart_dw_irq_err_enable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return;
}

static void uart_dw_irq_err_disable(const struct device *dev)
{
	ARG_UNUSED(dev);
	return;
}

static int uart_dw_irq_is_pending(const struct device *dev)
{
	struct uart_data *data = dev->data;

	return AlUart_ll_GetIntrStatus(data->uart);
}

static int uart_dw_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_dw_irq_callback_set(const struct device *dev,
					uart_irq_callback_user_data_t cb,
					void *cb_data)
{
	struct uart_data *data = dev->data;

	data->user_cb = cb;
	data->user_data = cb_data;
}

static int uart_dw_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	struct uart_data *data = dev->data;
	uint8_t num_tx = 0U;
	unsigned int key;

	while (!(AlUart_ll_GetThreState(data->uart)) && (AlUart_ll_IsFifosEnable(data->uart)));
	if (AlUart_ll_GetLsrThreState(data->uart)) {
		return -ENOTSUP;
	}

	key = irq_lock();

	while ((size - num_tx > 0) && !(AlUart_ll_GetLsrThreState(data->uart))) {
		AlUart_ll_SendByte(data->uart, tx_data[num_tx]);
		num_tx++;
	}

	irq_unlock(key);

	return num_tx;
}

static int uart_dw_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	struct uart_data *data = dev->data;
	uint8_t num_rx = 0U;

	if (!AlUart_ll_IsRxDataReady(data->uart)) {
		return -ENOTSUP;
	}

	while ((size - num_rx > 0) && AlUart_ll_IsRxDataReady(data->uart)) {
		rx_data[num_rx] = AlUart_ll_RecvByte(data->uart);
		num_rx++;
	}

	return num_rx;
}

#endif

static const struct uart_driver_api uart_dw_apb_driver_api = {
	.poll_in = uart_dw_poll_in,
	.poll_out = uart_dw_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_dw_fifo_fill,
	.fifo_read = uart_dw_fifo_read,
	.irq_tx_enable = uart_dw_irq_tx_enable,
	.irq_tx_disable = uart_dw_irq_rx_disable,
	.irq_tx_ready = uart_dw_irq_tx_ready,
	.irq_tx_complete = uart_dw_irq_tx_complete,
	.irq_rx_enable = uart_dw_irq_rx_enable,
	.irq_rx_disable = uart_dw_irq_rx_disable,
	.irq_rx_ready = uart_dw_irq_rx_ready,
	.irq_err_enable = uart_dw_irq_err_enable,
	.irq_err_disable = uart_dw_irq_err_disable,
	.irq_is_pending = uart_dw_irq_is_pending,
	.irq_update = uart_dw_irq_update,
	.irq_callback_set = uart_dw_irq_callback_set,
#endif
};

/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0
 */
static int uart_dw_apb_init(const struct device *dev)
{
	struct uart_data *data = dev->data;
	const struct uart_config *uart_cfg = dev->config;

	AlUart_ll_ResetUart0Bus();
	AlUart_ll_ResetUart1Bus();

	AlUart_ll_DisableAllIntr(data->uart);
	AlUart_ll_ResetTxFifo(data->uart);
	AlUart_ll_ResetRxFifo(data->uart);

	if (AlUart_ll_IsUartBusy(data->uart)) {
		// LOG_ERR("Al uart cannot set baudrate written while the UART is busy");
		return -EINVAL;
	} else {
		AlUart_ll_SetBaudRate(data->uart, uart_cfg->baudrate, data->clock);
		// AlUart_ll_SetDataWidth(data->uart, data->uart_cfg->data_bits);
		// AlUart_ll_SetParity(data->uart, data->uart_cfg->parity);
		// AlUart_ll_SetStopBitsLength(data->uart, data->uart_cfg->stop_bits);
	}

	if (AlUart_ll_IsUartBusy(data->uart)) {
	    // LOG_ERR("Al uart cannot set line control written while the UART is busy");
	    return -EINVAL;
	} else {
	    AlUart_ll_SetDataWidth(data->uart, uart_cfg->data_bits);
	    AlUart_ll_SetParity(data->uart, uart_cfg->parity);
	    AlUart_ll_SetStopBitsLength(data->uart, uart_cfg->stop_bits);
	}

	AlUart_ll_EnableFifo(data->uart, true);
	AlUart_ll_SetTxFifoThre(data->uart, AL_UART_TxFIFO_HALF_FULL);
	AlUart_ll_SetRxFifoThre(data->uart, AL_UART_RxFIFO_HALF_FULL);

	if (uart_cfg->flow_ctrl) {
		AlUart_ll_SetAutoFlowCtl(data->uart, true);
	}

#if defined(CONFIG_PM) || \
	defined(CONFIG_UART_INTERRUPT_DRIVEN) || \
	defined(CONFIG_UART_ASYNC_API)
	data->irq_config_func(dev);
#endif /* CONFIG_PM || CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API */

	return 0;
}


#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API) || \
	defined(CONFIG_PM)
#define DW_APB_UART_IRQ_HANDLER_DECL(index)				\
	static void uart_dw_irq_config_func_##index(const struct device *dev);
#define DW_APB_UART_IRQ_HANDLER(index)					\
static void uart_dw_irq_config_func_##index(const struct device *dev)	\
{									\
	IRQ_CONNECT(DT_INST_IRQN(index),				\
		DT_INST_IRQ(index, priority),				\
		uart_dw_isr, DEVICE_DT_INST_GET(index),		\
		0);							\
	irq_enable(DT_INST_IRQN(index));				\
}
#else
#define DW_APB_UART_IRQ_HANDLER_DECL(index) /* Not used */
#define DW_APB_UART_IRQ_HANDLER(index) /* Not used */
#endif

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API) || \
	defined(CONFIG_PM)
#define DW_UART_IRQ_HANDLER_FUNC(index)				\
	.irq_config_func = uart_dw_irq_config_func_##index,
#else
#define DW_APB_UART_IRQ_HANDLER_FUNC(index) /* Not used */
#endif

#define DW_APB_UART_INIT(index)						\
DW_APB_UART_IRQ_HANDLER_DECL(index)					\
									\
static struct uart_config uart_cfg_##index = {				\
	.baudrate  = DT_INST_PROP_OR(index, current_speed,		\
					 DW_APB_UART_DEFAULT_BAUDRATE),	\
	.parity    = DT_INST_ENUM_IDX_OR(index, parity,			\
					 DW_APB_UART_DEFAULT_PARITY),	\
	.stop_bits = DT_INST_ENUM_IDX_OR(index, stop_bits,		\
					 DW_APB_UART_DEFAULT_STOP_BITS),	\
	.data_bits = DT_INST_ENUM_IDX_OR(index, data_bits,		\
					 DW_APB_UART_DEFAULT_DATA_BITS),	\
	.flow_ctrl = DT_INST_PROP(index, hw_flow_control)		\
					? UART_CFG_FLOW_CTRL_RTS_CTS	\
					: UART_CFG_FLOW_CTRL_NONE,	\
};									\
									\
static struct uart_data uart_data_##index ={			\
	.uart = DT_INST_REG_ADDR(index),					\
	.clock = DT_INST_PROP_BY_PHANDLE(index, clocks, clock_frequency),	\
	DW_UART_IRQ_HANDLER_FUNC(index)				\
};														\
									\
									\
DEVICE_DT_INST_DEFINE(index,						\
			&uart_dw_apb_init,					\
			PM_DEVICE_DT_INST_GET(index),			\
		&uart_data_##index, &uart_cfg_##index,	\
			PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,		\
			&uart_dw_apb_driver_api);	\
DW_APB_UART_IRQ_HANDLER(index)

DT_INST_FOREACH_STATUS_OKAY(DW_APB_UART_INIT)
