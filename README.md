Лабораторна робота №3 Бугно Максим  АС-32  (варіант №1):
Підключення
![photo_1_2025-11-27_23-26-01](https://github.com/user-attachments/assets/f3abe576-4742-4477-9e08-020628f8db27)
Пини
<img width="1920" height="1033" alt="Снимок экрана (948)" src="https://github.com/user-attachments/assets/fa64e36c-8ba9-470a-93f0-f2ee7e609a50" />
```

static void mpu6500_update_gyro(void)
{
  uint8_t data[6];

  if (mpu6500_read(REG_GYRO_XOUT_H, data, sizeof(data)) == HAL_OK)
  {
    gyro_x = (int16_t)((data[0] << 8) | data[1]);
    gyro_y = (int16_t)((data[2] << 8) | data[3]);
    gyro_z = (int16_t)((data[4] << 8) | data[5]);

    gyro_dps_x = (float)gyro_x / GYRO_SENS_500DPS;
    gyro_dps_y = (float)gyro_y / GYRO_SENS_500DPS;
    gyro_dps_z = (float)gyro_z / GYRO_SENS_500DPS;
  }
}

static void mpu6500_log_gyro_swo(void)
{
  static uint32_t t_prev = 0u;
  uint32_t now = HAL_GetTick();

  if (now - t_prev > 200u)
  {
    t_prev = now;
    char buf[120];
    int n = snprintf(buf, sizeof(buf),
                     "gx=%6d gy=%6d gz=%6d  dps=[%7.2f %7.2f %7.2f]\n",
                     gyro_x, gyro_y, gyro_z,
                     (double)gyro_dps_x,
                     (double)gyro_dps_y,
                     (double)gyro_dps_z);
    if (n > 0) swo_print(buf);
  }
}
```
<img width="701" height="216" alt="Снимок экрана (949)" src="https://github.com/user-attachments/assets/0acbe6d4-895a-41a9-aaa9-b1c6bba45948" />
<img width="634" height="623" alt="Снимок экрана (950)" src="https://github.com/user-attachments/assets/a9bb8c33-01ae-4467-807d-325ad35e1098" />

