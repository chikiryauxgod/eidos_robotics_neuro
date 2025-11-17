# src/client/rcs_modbus_client.py
import logging
from dataclasses import dataclass
from typing import Dict, List
from pymodbus.client import ModbusTcpClient
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.constants import Endian
import yaml
import time

from ..utils.logger import get_logger

logger = get_logger(__name__)


@dataclass
class RCSConfig:
    """Конфигурация подключения и регистров манипулятора Eidos RCS."""
    host: str
    port: int
    timeout: float
    unit_id: int
    registers: Dict[str, int]          
    calibration: Dict[str, List[float]]  


class RCSModbusClient:
    """
    Клиент Modbus TCP для управления манипулятором Eidos Robotics A12
    через протокол, описанный в документации ErCode-Project и RCS Core 3.3.
    """

    def __init__(self, config_path: str = "config/config.yaml"):
        """
        Инициализация клиента.

        Args:
            config_path: путь к YAML-файлу с конфигурацией (по умолчанию config/config.yaml)

        См. ErCode-Project → раздел 3.2 «RCS Connection» (стр. 10–16)
        """
        self.config = self._load_config(config_path)
        self.client = ModbusTcpClient(
            host=self.config.host,
            port=self.config.port,
            timeout=self.config.timeout,
            retries=3,
            retry_on_empty=True,
        )
        self._connect()

    def _load_config(self, path: str) -> RCSConfig:
        """Загрузка конфигурации из YAML-файла."""
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}

        return RCSConfig(
            host=data["rcs"]["host"],
            port=data["rcs"]["port"],
            timeout=data["rcs"]["timeout"],
            unit_id=data["rcs"]["unit_id"],
            registers=data["registers"],
            calibration=data["calibration"]
        )

    def _connect(self):
        """Установка соединения по Modbus TCP."""
        if not self.client.connect():
            raise ConnectionError(f"Failed to connect to RCS: {self.config.host}:{self.config.port}")
        logger.info("Connected to RCS via Modbus TCP")

    def _write_float(self, address: int, value: float):
        """Запись 32-битного float в два последовательных регистра (Big-Endian byteorder, Little-Endian wordorder)."""
        builder = BinaryPayloadBuilder(byteorder=Endian.BIG, wordorder=Endian.LITTLE)
        builder.add_32bit_float(value)
        payload = builder.to_registers()
        self.client.write_registers(address, payload, unit=self.config.unit_id)

    def _write_int(self, address: int, value: int):
        """Запись целого числа в один регистр."""
        self.client.write_register(address, value, unit=self.config.unit_id)

    def _read_int(self, address: int) -> int:
        """Чтение одного целого числа из holding-регистра."""
        result = self.client.read_holding_registers(address, 1, unit=self.config.unit_id)
        if result.isError():
            raise ValueError(f"Error reading register {address}: {result}")
        return result.registers[0]

    def reset_errors(self) -> None:
        """
        Сброс всех ошибок и аварийных состояний манипулятора.

        Формирует положительный импульс в регистре i1.100.0.
        См. ErCode-Project → п. 4.8.1 «Элемент окна редактирования сигналов» (стр. 22)
        и виджет «Сигналы» в RCS Core → стр. 40.
        """
        self._write_int(self.config.registers['reset_errors'], 1)
        time.sleep(0.1)
        self._write_int(self.config.registers['reset_errors'], 0)
        logger.info("Errors have been reset")

    def enable_drives(self) -> None:
        """
        Включение приводов (разрешение движения по всем осям).

        Записывает 1 в регистр i1.100.1.
        См. ErCode-Project → п. 4.6 «Действия с подключенными соединениями» (стр. 34).
        """
        self._write_int(self.config.registers['enable_drives'], 1)
        logger.info("Drives enabled")

    def start_program(self, program_id: int) -> None:
        """
        Запуск программы по её номеру.

        Последовательно записывает номер программы в i4.107.0 и формирует
        положительный импульс в регистре «Старт программы» (i4.107.x).
        См. ErCode-Project → п. 14.1.2 «Обзор программного редактора» (стр. 269).
        """
        self._write_int(self.config.registers['program_number'], program_id)
        self._write_int(self.config.registers['start_program'], 1)
        time.sleep(0.1)
        self._write_int(self.config.registers['start_program'], 0)
        logger.info(f"Started program #{program_id}")

    def move_to_xyz(self, x: float, y: float, z: float) -> None:
        """
        Линейное перемещение TCP в системе координат Base frame.

        Записывает координаты в выходные регистры o4.200–o4.202
        и запускает программу №1 (обычно это линейное перемещение).
        См. RCS Simulator 3.3 → раздел 10 «Работа с объектами» (стр. 56–59)
        и ErCode-Project → калибровка Base frame (стр. 196–197).
        """
        self._write_float(self.config.registers['target_x'], x)
        self._write_float(self.config.registers['target_y'], y)
        self._write_float(self.config.registers['target_z'], z)
        self.start_program(1)
        logger.info(f"Command: move to ({x:.2f}, {y:.2f}, {z:.2f})")

    def go_home(self) -> None:
        """
        Возврат манипулятора в домашнюю позицию.

        Координаты берутся из секции calibration.home_position в config.yaml.
        """
        pos = self.config.calibration['home_position']
        self.move_to_xyz(pos[0], pos[1], pos[2])
        logger.info("Returning to home position")

    def calibrate_base(self) -> None:
        """
        Ручная трёхточечная калибровка Base frame с помощью конусов.

        Последовательно запрашивает у оператора подвести TCP к трём конусам.
        После ввода всех точек калибровку необходимо завершить в HMI RCS.
        См. ErCode-Project → стр. 196–197 и RCS Simulator 3.3 → раздел 10.1 (стр. 56).
        """
        logger.info("Starting Base frame calibration (3-point method)...")
        for i, point in enumerate(self.config.calibration['cone_positions']):
            x, y, z = point
            logger.info(f"Move TCP to cone {i+1} at ({x:.3f}, {y:.3f}, {z:.3f}) and press Enter")
            input("Press Enter when in position...")
        logger.info("Calibration points recorded. Complete calibration in RCS HMI.")

    def get_status(self) -> Dict[str, str]:
        """
        Чтение слова состояния манипулятора.

        Читает регистр i2.300.0 (виджет «Глобальные переменные» → «Состояние»).
        Возвращает десятичное и двоичное представление.
        См. RCS Core → стр. 44–47.
        """
        status = self._read_int(self.config.registers['status_word'])
        return {"status_word": status, "binary": bin(status)}

    def close(self) -> None:
        """Корректное закрытие Modbus-соединения."""
        self.client.close()
        logger.info("Modbus connection closed")