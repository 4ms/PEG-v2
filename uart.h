uint8_t get_next_uart_data(uint8_t *data);

void init_uart(uint16_t ubbr);

void usart_putc(unsigned char c);

void enable_usart_TX(void);
void disable_usart_TX(void);
