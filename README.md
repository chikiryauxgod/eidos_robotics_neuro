# Eidos Robotics A12 — Детекция объекта в рабочей зоне и подвод TCP к цели


## Цель проекта

Реализация системы технического зрения, которая:
- Обнаруживает произвольный объект в рабочей зоне манипулятора с помощью YOLOv8
- Вычисляет координаты центра объекта в системе координат Base frame
- Перемещает TCP манипулятора Eidos Robotics A12 точно к обнаруженному объекту

## Архитектура решения

Камера → OpenCV → YOLOv8 → координаты (x, y, z) → Modbus/TCP → Eidos RCS → движение TCP

## Установка и запуск


# 1. Клонировать репозиторий
```bash
git clone https://github.com/chikiryauxgod/eidos_robotics_neuro.git
cd eidos_robotics_neuro
```

# 2. Создать и активировать виртуальное окружение
```bash
python -m venv venv
source venv/bin/activate
```

# 3. Установить зависимости
```bash
pip install -r requirements.txt
```

# 4. Запустить демо 
``` bash
python -m src.demo.main
```

## Авторы

Проект выполнен студентами группы ИПР-22-1б:

- Малых Игорь      – https://github.com/faweddd
- Петров Матвей   – https://github.com/yvelyu
- Семенов Лев     – https://github.com/chikiryauxgod 
