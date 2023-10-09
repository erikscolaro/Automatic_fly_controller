# Automatic_fly_controller
The aim of this project is to develop a controller for the Arduino PRO MINI (ATmega328) capable of autonomously piloting a small glider.

Partial list of components
-
- microcontroller: arduino PRO MINI atmega328 3.3v/5v (https://it.aliexpress.com/item/32821902128.html?spm=a2g0o.order_list.order_list_main.46.4d793696Alp33B&gatewayAdapt=glo2ita)
- servomotors: SG90 x4
- propulsion control: L298N      (https://it.aliexpress.com/item/4001050948286.html?spm=a2g0o.order_list.order_list_main.21.4d793696Alp33B&gatewayAdapt=glo2ita)
- propulsion: brushless drone motor 8mm*20mm DC 3V 3.7V 43000RPM    (https://it.aliexpress.com/item/1005004949100539.html?spm=a2g0o.order_list.order_list_main.40.4d793696Alp33B&gatewayAdapt=glo2ita)
- gps: GY-NEO6MV2 with active antenna                                (https://it.aliexpress.com/item/1005004447572702.html?spm=a2g0o.order_list.order_list_main.51.4d793696Alp33B&gatewayAdapt=glo2ita)
- gyroscope and accelerometer: MPU6050      

Goal of the project
-
The objective of the project is to be able to create a controller to fly a small glider with the supplied components described above. Two of the servomotors are used to modify the angle of the plane's flaps, while the other two are used to open the exhaust and landing doors.
Ideally, the aircraft should be capable of:
1. understand when it was launched with the accelerometer, consequently start the take-off sequence;
2. use the GPS, accelerometer and gyroscope to pilot the plane and make it reach a certain position;
3. having reached the position, open the cargo door in flight, delivering a package equipped with a parachute;
4. carry out the necessary maneuvers to return to the starting position;
5. Having reached the home base, open the second door which will contain a parachute to make the landing.
