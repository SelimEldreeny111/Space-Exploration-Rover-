# MISSION X demo video — narration script

There are two editions of the video, each 2 min 56 s long:
`MissionX_Rover_Demo.mp4` has English captions and `MissionX_Rover_Demo_AR.mp4`
has Arabic captions with a right-to-left layout. Both use a procedurally
generated soundtrack, so they contain no third-party music. The Arabic credits
spell the team's names as written in `Documentation/show/Rover.txt`. **Please
check them.** For example, "عمر عبدالحكيم عبدالاه" and "براء محمد جبر" are
copied as-is. To add a human voice-over like the 2023 news piece, record the lines
below (English **or** Arabic) against the timings. Then mix the recording under
the music with:

```bash
ffmpeg -i MissionX_Rover_Demo.mp4 -i voice.wav -filter_complex "[0:a]volume=0.30[m];[m][1:a]amix=inputs=2:duration=first:normalize=0[a]" -map 0:v -map "[a]" -c:v copy -c:a aac -b:a 192k MissionX_Rover_Demo_voice.mp4
```

| Time | Scene | English | العربية |
|------|-------|---------|---------|
| 0:00–0:09 | Title | This is MISSION X, a space exploration rover designed and built by students of Beni-Suef University. | هذه مهمة «ميشن إكس»: مركبة لاستكشاف الفضاء صمّمها ونفّذها طلاب جامعة بني سويف. |
| 0:09–0:16 | Institution | It is a graduation project of the Faculty of Navigation Science and Space Technology, in cooperation with the Egyptian Space Agency. | وهي مشروع تخرج بكلية علوم الملاحة وتكنولوجيا الفضاء، بالتعاون مع وكالة الفضاء المصرية. |
| 0:16–0:27 | Mission | Its mission is to explore unknown terrain on its own: build a map, navigate safely, measure the environment, and collect soil samples. | مهمتها استكشاف بيئة مجهولة ذاتيًا: ترسم خريطة، وتتحرك بأمان، وتقيس ظروف البيئة، وتجمع عينات من التربة. |
| 0:27–0:40 | Structure | A rocker–bogie suspension with a differential pivot keeps all six wheels on the ground. The body is laser-cut aluminium, and the rover weighs 36.5 kilograms. | نظام تعليق «روكر بوجي» مع محور تفاضلي يُبقي العجلات الست على الأرض، وهيكل من الألومنيوم المقطوع بالليزر، ووزن كلي ستة وثلاثون كيلوجرامًا ونصف. |
| 0:40–0:49 | Mobility | Six geared motors with encoders drive the rover at up to 0.66 metres per second and let it climb slopes of about 27 degrees. | ستة محركات بتروس مزودة بمشفرات تدفع المركبة بسرعة تصل إلى ستة وستين سنتيمترًا في الثانية، وتصعد منحدرات تقارب سبعًا وعشرين درجة. |
| 0:49–0:58 | Power | A 50-watt solar panel charges 540 watt-hours of lithium-ion batteries, enough for almost seven hours of typical operation. | لوح شمسي بقدرة خمسين وات يشحن بطاريات ليثيوم سعتها خمسمائة وأربعون وات ساعة، تكفي قرابة سبع ساعات من التشغيل المعتاد. |
| 0:58–1:16 | Brain / ROS | A Raspberry Pi running ROS builds a map of the lab with a 360-degree LiDAR, then navigates autonomously between benches and chairs. | حاسوب «راسبيري باي» يعمل بنظام ROS يرسم خريطة المعمل بحساس ليدار دوّار، ثم يتنقّل ذاتيًا بين المناضد والكراسي. |
| 1:16–1:27 | Payload | The payload measures gases, temperature, humidity, pressure, attitude, soil moisture, rain and metals, and streams every reading. | تقيس الحمولة العلمية الغازات والحرارة والرطوبة والضغط والاتجاه ورطوبة التربة والمطر والمعادن، وتُرسل كل القراءات لحظيًا. |
| 1:27–1:36 | Arm | A six-axis robotic arm reaches the ground to collect samples. | ذراع روبوتية بست درجات حرية تصل إلى الأرض لجمع العينات. |
| 1:36–1:49 | Ground station | A 433-megahertz radio link carries a CRC-protected protocol to the LabVIEW and mobile ground stations, and now to a new web ground station. | وصلة راديو بتردد أربعمائة وثلاثة وثلاثين ميجاهرتز تنقل بروتوكولًا محميًا بفحص الأخطاء إلى محطتي «لاب فيو» والهاتف، وإلى محطة أرضية جديدة على المتصفح. |
| 1:49–2:03 | Digital twin | In 2026 we rebuilt the software on ROS 2 and created a digital twin. In simulation, LiDAR scan matching keeps the rover within about ten centimetres of the truth. | في عام ألفين وستة وعشرين أعدنا بناء البرمجيات على ROS 2 وأنشأنا توأمًا رقميًا؛ وفي المحاكاة تُبقي مطابقة مسح الليدار خطأ الموقع في حدود عشرة سنتيمترات. |
| 2:03–2:21 | Results | Fusing ultrasonic sensors removed every collision with low obstacles. The new protocol executed no wrong command in half a million trials, and gyro fusion cut odometry drift to half a percent. | دمج الحساسات فوق الصوتية أزال كل التصادمات مع العوائق المنخفضة، والبروتوكول الجديد لم يُنفّذ أي أمر خاطئ في نصف مليون تجربة، ودمج الجيروسكوب خفّض انحراف الملاحة إلى نصف في المائة. |
| 2:21–2:31 | Numbers | MISSION X in numbers. | «ميشن إكس» بالأرقام. |
| 2:31–2:41 | Deliverables | The project delivers a research paper, a ROS 2 software stack, new firmware, an interface control document, and a web ground station. | يقدّم المشروع ورقة بحثية، ومنظومة برمجيات ROS 2، وبرمجيات جديدة للمتحكمات، ووثيقة للتحكم في الواجهات، ومحطة أرضية على الويب. |
| 2:41–2:56 | Team | MISSION X: ten students, two supervisors, one rover. Thank you. | «ميشن إكس»: عشرة طلاب، ومشرفان، ومركبة واحدة. شكرًا لكم. |

## Sources used in the video

Only the team's own material is used: report photos, presentation clips
(`media3.mp4` rover run and `media4.mp4` arm, with the editor watermark cropped),
`navigation.mp4`, `sensor.mp4`, the `show/` photos, ground-station screenshots,
the paper figures and a re-run of the simulator. The Perseverance animation in
the 2023 presentation (`media1.mp4`) belongs to a third party and is **not**
used.
