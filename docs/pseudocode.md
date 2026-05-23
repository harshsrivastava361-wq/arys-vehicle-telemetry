# Pseudocode — Real-Time Vehicle Telemetry System
Arys Garage Pvt. Ltd. — Assignment Q2
Candidate: Harsh Kumar Srivastava

## Task 1: Sensor Acquisition (Priority 5 — 100Hz)

    TASK vTaskSensorAcquisition:
        INIT:
            roll  = 0.0
            pitch = 0.0
            alpha = 0.98
            dt    = 0.01 seconds

        LOOP every 10ms:
            raw = I2C_Read(MPU6050, address=0x68)

            ax = raw.accel_x * 9.81 / 16384
            ay = raw.accel_y * 9.81 / 16384
            az = raw.accel_z * 9.81 / 16384
            gx = raw.gyro_x  * pi / (180 * 131)
            gy = raw.gyro_y  * pi / (180 * 131)

            accel_roll  = atan2(ay, az) * 180/pi
            accel_pitch = atan2(-ax, az) * 180/pi
            roll  = alpha*(roll  + gx*dt*180/pi) + (1-alpha)*accel_roll
            pitch = alpha*(pitch + gy*dt*180/pi) + (1-alpha)*accel_pitch

            g_force   = sqrt(ax*ax + ay*ay + az*az) / 9.81
            adc_val   = ADC_Read(channel=0)
            wheel_rpm = (adc_val / 4095) * MAX_RPM
            speed_kmh = (wheel_rpm * pi * 0.56 * 60) / 1000

            fused = {timestamp, ax, ay, az, roll, pitch, g_force, rpm, speed}
            QUEUE_SEND(xIMUQueue, fused)
            WAIT_UNTIL next 10ms tick

## Task 2: GPS Parsing (Priority 4)

    TASK vTaskGPSParsing:
        LOOP forever:
            sentence = UART_ReadLine(port=GPS, timeout=2000ms)

            IF sentence == TIMEOUT:
                FAULT_Set(FAULT_GPS_LOSS)
                CONTINUE

            IF sentence starts with $GPRMC:
                fields = SPLIT(sentence, comma)
                IF fields[2] == A (valid fix):
                    latitude  = PARSE_DEGREES(fields[3], fields[4])
                    longitude = PARSE_DEGREES(fields[5], fields[6])
                    speed_kmh = PARSE_FLOAT(fields[7]) * 1.852
                    heading   = PARSE_FLOAT(fields[8])
                    FAULT_Clear(FAULT_GPS_LOSS)
                    QUEUE_SEND(xGPSQueue, gps_data)

## Task 3: CAN Handler (Priority 3)

    TASK vTaskCANHandler:
        LOOP forever:
            frame.id   = 0x100
            frame.data = {speed, rpm, g_force, fault_flags}
            CAN_Transmit(frame)

            rx_frame = CAN_Receive(timeout=1000ms)
            IF rx_frame == TIMEOUT:
                FAULT_Set(FAULT_CAN_TIMEOUT)
                CAN_BusOff_Recovery()
            ELSE:
                FAULT_Clear(FAULT_CAN_TIMEOUT)
                QUEUE_SEND(xCANQueue, rx_frame)

            IF fault_register != 0:
                CAN_Transmit(id=0x7FF, data=fault_register)
            WAIT 10ms

## Task 4: Data Logger (Priority 2)

    TASK vTaskDataLogger:
        INIT:
            result = SD_Mount() and SD_OpenFile()
            IF result == FAIL:
                FAULT_Set(FAULT_SD_FAIL)
                use_ram_buffer = TRUE

        LOOP forever:
            entry = QUEUE_RECEIVE(xLogQueue, timeout=100ms)
            line  = FORMAT_CSV(entry)

            IF use_ram_buffer == FALSE:
                MUTEX_TAKE(xSDCardMutex)
                result = SD_Write(line)
                IF result == FAIL:
                    use_ram_buffer = TRUE
                IF every 10 entries:
                    SD_Flush()
                MUTEX_GIVE(xSDCardMutex)
            ELSE:
                ram_buffer[head] = entry
                head++

## Task 5: Wireless Telemetry (Priority 1)

    TASK vTaskWirelessTelemetry:
        LOOP forever:
            packet = BUILD_PACKET(speed, rpm, gforce, lat, lon, faults)

            IF FAULT_IsSet(FAULT_SD_FAIL):
                FOR each entry in ram_buffer:
                    WIRELESS_Send(entry)
            ELSE:
                WIRELESS_Send(packet)
            WAIT 100ms

## Task 6: Fault Monitor (Priority 6 — HIGHEST)

    TASK vTaskFaultMonitor:
        LOOP every 50ms:
            now = GET_TICK_COUNT()

            IF (now - last_gps_tick) > 2000ms AND GPS fault not set:
                FAULT_Set(FAULT_GPS_LOSS)
                Telemetry_SetMode(DEAD_RECKONING)

            IF (now - last_imu_tick) > 500ms AND IMU fault not set:
                FAULT_Set(FAULT_IMU_DISC)
                SensorFusion_Disable()

            IF (now - last_can_tick) > 1000ms AND CAN fault not set:
                FAULT_Set(FAULT_CAN_TIMEOUT)
                CAN_BusOff_Recovery()

            IF fault_register != 0:
                CAN_Send(id=0x7FF, data=fault_register)
            WAIT 50ms

## Fault Register Bitmask

    Bit 0 = FAULT_GPS_LOSS     (GPS fix lost > 2000ms)
    Bit 1 = FAULT_IMU_DISC     (IMU no data > 500ms)
    Bit 2 = FAULT_CAN_TIMEOUT  (CAN no frame > 1000ms)
    Bit 3 = FAULT_SD_FAIL      (SD write error)

## Inter-Task Queue Summary

    xIMUQueue  depth 20 : SensorAcq  sends to DataLogger
    xGPSQueue  depth 10 : GPSParser  sends to DataLogger
    xCANQueue  depth 15 : CANHandler sends to DataLogger
    xLogQueue  depth 50 : All tasks  send  to DataLogger
    xSDCardMutex        : Protects SD card from concurrent access
