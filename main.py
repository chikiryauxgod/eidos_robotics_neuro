import cv2
import logging
from ultralytics import YOLO
from src.utils.logger import get_logger
from src.client.rcs_modbus_client import RCSModbusClient
import time

logger = get_logger('MainLogger')

def main():
    try:
        rcs = RCSModbusClient()
       
    except Exception as e:
        logger.error(f"ModBus client error: {e}")
        
    
    try:
        rcs.reset_errors()
        rcs.enable_drives()
        rcs.go_home()
        time.sleep(2)
        model = YOLO("yolov8n.pt")
    

    except Exception as e:
        logger.error(f"Runtime error: {e}")
    finally:
        rcs.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()