* SPI
<pre><code>
           |                 |
           |             P3.0|-> Data Out (UCB0SIMO)
           |                 |
           |             P3.1|<- Data In (UCB0SOMI)
           |                 |
Slave(CS)<-|P1.2         P3.2|-> Serial Clock Out (UCB0CLK)

</code></pre>

* 串口 - 蓝牙
<pre><code>
           |              XIN|-
           |                 | 32kHz
           |             XOUT|-
           |                 |
           |     P3.3/UCA0TXD|------------>
           |                 | 9600 - 8N1
           |     P3.4/UCA0RXD|<------------

</code></pre>

* IIC - OLED
<pre><code>
           |P4.1/UCB0SDA  XIN|-
           |                 |
           |             XOUT|-
           |P4.2/UCB0SCL     |
           |                 |
</code></pre>

* AD -  序列采样
<pre><code>
           |   P6.0/ADC12MEM0|<------------
           |                 | 
           |   P6.1/ADC12MEM1|<------------
</code></pre>
