* ADG5405
<pre><code>
         P3.6->|A1               |
         P3.5->|A0               |

</code></pre>

* TS5N214
<pre><code>
         P1.5->|1/OE             |
         P1.4->|2/OE             |
         P1.3->|S1               |
         P1.2->|S0               |

</code></pre>

* SPI
<pre><code>
               |                 |
               |             P3.0|-> Data Out (UCB0SIMO)
               |                 |
               |             P3.1|<- Data In (UCB0SOMI)
               |                 |
    Slave(CS)<-|P2.5         P3.2|-> Serial Clock Out (UCB0CLK)
           DA->|OUTA 4

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
           |                 |
            
</code></pre>


