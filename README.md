# STM32 micro-ROS Encoder Speed Publisher

STM32F446RE kullanılarak optik enkoderden motor hızının (RPM) ölçülmesi, 
hareketli ortalama filtresi ile stabilize edilmesi ve micro-ROS üzerinden
ROS 2 ortamına yayınlanması.

## Encoder ve Filtre Parametreleri Özeti

| Parametre                     | Değer            | Açıklama |
|------------------------------|------------------|----------|
| Enkoder Tipi                 | Optik, tek kanal | Yükselen kenar sayımı |
| Disk Delik Sayısı            | 20               | 1 turdaki darbe sayısı |
| Timer                        | TIM2             | 32-bit donanımsal sayaç |
| Timer Modu                   | External Clock 1 | Harici darbe ile sayım |
| Sayaç Çözünürlüğü            | 32-bit           | 0 – 4.294.967.295 |
| Örnekleme Periyodu           | 100 ms           | 10 Hz hız hesaplama |
| Hız Birimi                   | RPM              | Dakikadaki devir sayısı |
| Ham Hız Hesabı               | ΔCount / 20 × 600| Darbe → RPM dönüşümü |
| Filtre Tipi                  | Hareketli Ortalama | Moving Average |
| Filtre Pencere Boyutu (N)    | 5                | Son 5 ölçüm |
| Filtre Yapısı                | FIFO (Circular)  | Dairesel tampon |
| Yayınlanan Topic             | `/wheel_speed`   | micro-ROS publisher |
| Mesaj Tipi                   | `std_msgs/Float32` | ROS 2 uyumlu |

## Teknik Doküman
- [Encoder hız hesaplama ve filtreleme teknik notu](Docs/Encoder_Teknik_Not.pdf)


## Sistem Testi

![Encoder Test](media/encoder_test.jpg)

## Kullanılan Donanım
- STM32F446RE
- Optik enkoder (20 delik)
- DC motor

## İletişim
micro-ROS UART (USART6)
