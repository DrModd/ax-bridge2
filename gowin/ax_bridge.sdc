# Audio MCLK: 49.152 MHz (or 45.1584 MHz - timed at the faster one).
create_clock -name mclk_in -period 20.345 [get_ports {mclk_in}]

# Front-end clock: the selected input BCLK after the 5:1 mux (in_mux), net
# m_bclk in top.v. bclk_rx and the FIFO write side run on it. Worst case
# PCM 768 kHz x 64 = 49.152 MHz. The five BCLK pins only feed the mux, so
# this one clock covers every input.
create_clock -name m_bclk -period 20.345 [get_nets {m_bclk}]

# 50 MHz service clock (I2C, S/PDIF receiver, meter, clock manager).
create_clock -name sys_clk -period 20.000 [get_ports {sys_clk}]
# The three domains meet only in the Gray-coded FIFOs and in 2-FF
# synchronisers (control levels are taken after 4 stable clocks).
set_clock_groups -asynchronous -group [get_clocks {sys_clk}] -group [get_clocks {mclk_in}] -group [get_clocks {m_bclk}]
