# FlexiTac reference (collected by the orchestrator on 2026-09-25)

**Sources**
- Project page: https://flexitac.github.io
- Paper: Huang & Li, Columbia, arXiv:2604.28156
- Hardware repo: github.com/FlexiTac/FlexiTac_Hardware_Repo (firmware `32_16_fast.ino`, host `fast_32_16.py`)
- Tutorial (Google Doc): exported to `work/tactile/ref/tutorial.txt`
- Local copies of all of the above: `work/tactile/ref/`
- **License: CC BY-NC 4.0** (non-commercial)

## Sensor (V2, current)
- Two FPCs, top and bottom: flex PCB **0.2 mm**, gold fingers 0.3 mm thick. The top FPC carries the row electrodes and the bottom the column electrodes, or the other way round.
- Piezoresistive film between the FPCs: **Velostat/Linqstat** (Adafruit 1361, about 0.1 mm).
- Each FPC is laminated with **adhesive laminating sheets**, and the edges are **sealed with polyimide (Kapton) tape**.
- The whole stack is roughly 0.6–0.8 mm; the page gives no exact figure. Budget **0.8 mm**, plus an optional protective skin.
- Standard V2 piece: **32 × 12 taxels at 2 mm pitch**, about 64 × 24 mm active, about $2.50 per pair at 100 units.
- V1 was 16 × 16 at 3 mm pitch.
- Custom shapes are made by designing your own top and bottom FPC Gerbers. The FlexiTac pitch is 2 mm.
- Durability: about 1 year, and "thousands of demonstrations".

## Readout
- **"Reading Board 32x16"**: an Arduino Nano (ATmega328), **1 analog mux** (16 rows, 4 select pins plus inhibit) and **4 shift registers** (32 columns, data plus clock). All taxels go through the single ADC input A0, with a 10-bit reading reduced to 8 bits.
- **0.5 mm-pitch FFC connectors**: 16-pin for the rows and 32-pin for the columns. The FFC extenders are 20-pin and 40-pin Adafruit parts, or bulkier AliExpress easy-install ones.
- One board reads **one 16 × 32 matrix (512 taxels)**. A "Reading Board 32x32" (1024 taxels) also exists, marked as a development board.
- Output is USB serial (mini-USB on the Nano) at **2,000,000 baud**. Each frame is the header `0xAA 0x55` followed by 512 bytes, one byte per taxel.
- Host side: PyPI `flexitac` (`FlexiTacSensor`), with baseline subtraction, a threshold and normalisation.
- Board dimensions are not published: the Nano is 18 × 45 mm, and the reading board is roughly Nano-sized plus its FFC connectors. Model it as about 50 × 25 × 15 mm with the Nano plugged in, and say that this is an assumption.
