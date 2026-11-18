# SOARM-ASTRA

ต้นแบบ offline สำหรับ SO-101 ที่รับ goal ภาษาไทยครั้งเดียว แล้วหยิบก้อนที่ระบุไปวางในถาดภายในฉาก tabletop ที่รองรับ เป้าหมายการทดลองบนเครื่องจริงคือทดสอบคำสั่งและ layout ใหม่โดยไม่ train ใหม่ รอบนี้เริ่มที่ typed event หลังขั้นรับเสียง

## รันจากราก repo

ใช้ Python 3.12 และ standard library ได้เลย

```sh
python3 -B -m unittest discover -s tests -v
python3 -B -m soarm_astra 'ช่วยเอาก้อนแดงใส่ถาด'
python3 -B -m soarm_astra --trace 'หยิบก้อนสีฟ้าไปวางในถาด'
```

Trace เป็น JSONL พร้อมเวลา fake clock ในหน่วย ms, revision, obs_id (ฟิลด์ `obs`), measured checkpoint และ tick writes เมื่อสถานะเป็น `ask` อ่านคำถามได้จาก `last_decision` ของ CLI

## สิ่งที่รันได้

- `VoiceIngress` รับ typed goal/override/stop ใน process เดียวกับ control loop
- `FixturePlanner` ทำหน้าที่แทน Astra ในฉากจำลอง อ่านตำแหน่งจากพิกเซลและสร้าง Cartesian waypoints ให้แต่ละช่วง มี revision, subgoals และเกณฑ์สำเร็จ การตรวจหลังวางใช้ภาพของก้อนในถาดและมือว่าง
- `ControlLoop` ตรวจ schema, อายุภาพและ encoder, preflight ทั้งช่วง, สั่งทีละ tick, ตรวจ tracking, จำกัด retry และจัดการ interrupt พร้อม hold/ACK/settled
- กล้องบนและกล้องข้อมือสังเคราะห์สร้างภาพคนละเวลา โลกจำลองตอบสนองต่อ setpoint และการหยิบจริงใน fixture กรณีของตกหรือวางพลาดจะเปลี่ยนตำแหน่งที่ planner เห็นในรอบถัดไป
- คำสั่งที่ fixture รองรับระบุสีฟ้าหรือสีแดงหนึ่งสีและถาดหนึ่งใบ ความสามารถด้านภาษาของโมเดลและผลการทดสอบ held-out ต้องวัดแยกต่างหาก

[สถาปัตยกรรมและ adapter](docs/architecture.md) · [Scenario matrix และข้อจำกัด](docs/evaluation.md)

การรันตัวอย่างไม่ต้องตั้ง key รอบต่อไปที่ทดสอบ GPT Live/Astra ให้ตั้ง `OPENAI_API_KEY` ผ่าน secret/env ของ executor เก็บ key นอกแชตและ Git โมเดลที่ใช้เขียนโค้ดใน Codex แยกจาก OpenAI runtime ที่ต้องทดสอบในแอป
