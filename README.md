This GPIO  Bare-metal drivers  offer high-level APIs for GPIO configuration and handling, which abstract away the direct interaction with hardware registers. These drivers simplify development by providing a standardized interface, but at the cost of increased overhead. This is  generally suitable for non-real-time applications where performance and precision are not critical concerns. In contrast, low-level drivers provide minimal abstraction, allowing direct access to hardware registers for more precise control. which are ideal for real-time applications where low latency and fine-grained timing are essential.
***********************************************************************

General Overview:
![WhatsApp Image 2024-09-08 at 1 49 56 AM (1)](https://github.com/user-attachments/assets/7db64caa-e52a-4d16-a7a7-5b84ad54b12b)

**********************************************************************


Output Modes
1) Push-Pull Configuration
2) Open Drain with Pull-UP Configuration
![WhatsApp Image 2024-09-08 at 1 49 54 AM](https://github.com/user-attachments/assets/86724d9c-6223-4244-b806-44da40caac77)

**********************************************************************


Input Mode with Interrup Configuration
![WhatsApp Image 2024-09-08 at 1 49 57 AM](https://github.com/user-attachments/assets/d0b396e5-7261-4e1f-8810-3bcc0660488f)






**********************************************************************

Note: This Code is Part of Learning STM32 Bare Metal Driver Development from Udemy! I have written these drivers from scratch and am learning from my course.
Any Errors, please correct me.
Thank you.
