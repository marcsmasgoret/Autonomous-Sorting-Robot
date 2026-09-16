**1. Problem Statement:**


The objective of this project is to design and develop a fully autonomous mobile robot capable of operating within a defined arena to locate, collect, and sort target objects under a strict 120-second time constraint. The robot must identify and collect green erasers while distinguishing them from non-target objects, then return to a designated home base where the objects are deposited and correctly sorted to maximize scoring performance. The system must function without any external control or intervention, relying entirely on onboard sensing, actuation, and control systems. The design is constrained by factors including limited operation time, physical size restrictions, onboard power supply, and the requirement for reliable autonomous navigation and object handling. The robot must efficiently traverse the arena, avoid obstacles, and consistently return to the base for deposition, while maintaining accurate sorting of collected objects. Scoring is directly dependent on both the quantity of target objects collected and the accuracy of their classification, making both efficiency and precision critical performance metrics. This problem reflects real-world engineering challenges in autonomous robotics, such as industrial material sorting, warehouse automation, and hazardous environment operations, where systems must operate independently and reliably without human intervention. The difficulty of the task lies in the integration of multiple subsystems—navigation, collection, sensing, and sorting—within tight constraints, while ensuring consistent performance despite uncertainties such as object positioning, sensor limitations, and mechanical interactions.

**2. Design Requirements & Constraints:**

The design requirements were developed to ensure that both the mobile robot and the stationary sorting system operate effectively within the given project constraints. The system integrates navigation, collection, and sorting subsystems, each with specific performance targets. Key requirements such as autonomous operation, reliable navigation using an IMU, and accurate color-based sorting guided the selection of components and overall system architecture. Feasibility of the design was validated through iterative prototyping and testing. Mechanical adjustments, including improved scoop clearance and weight reduction, enhanced system performance and reduced actuator load. Additionally, the use of a stationary sorting system improved classification reliability by providing controlled sensing conditions. Overall, the design meets the functional requirements within the constraints of time, power, and system complexity. Key quantitative constraints include a maximum operation time of 120 seconds, operation within a ~15 m² arena, and achieving a sorting accuracy target of at least 90%. [Design Requirements Table.xlsx](https://github.com/user-attachments/files/26483405/Design.Requirements.Table.xlsx)



**3. Program Plan:**

The project was completed over an eight-week period, following a structured and iterative development process. The timeline began with concept generation and selection, followed by mechanical design and CAD modeling of the chassis and scoop system. Prototype fabrication using 3D printing was then carried out, enabling initial testing of the drive, collection, and sorting subsystems. Testing played a critical role in identifying performance limitations, particularly in the scoop mechanism and object handling process. These issues led to a second design iteration, where mechanical refinements such as improved clearances and weight reduction were implemented. Programming and sensor integration were conducted concurrently with testing, reflecting the iterative nature of the development process. The critical path of the project included CAD modeling, prototype fabrication, and system testing, as delays in these stages directly impacted subsequent development. Overlapping tasks, such as programming and system testing, allowed for continuous refinement and improved overall efficiency. The project timeline also accounted for potential schedule variations and risks. Mechanical issues encountered during the initial prototype required redesign and reprinting, which introduced delays. To mitigate this, buffer time was effectively incorporated within the testing and iteration phases, allowing adjustments to be made without significantly affecting the final deadline. This ensured that system integration, final testing, and documentation were completed in time for the final showcase.
[Weekly Gantt Chart.xlsx](https://github.com/user-attachments/files/26482796/Weekly.Gantt.Chart.xlsx)


**4. Concept Generation:**

   
The system design was developed by decomposing the overall problem into key subsystems, including drivetrain, navigation, object collection, sorting, and homing. For each subsystem, multiple candidate concepts were generated using a combination of engineering reasoning and structured design approaches. This ensured that the final solution was not based on a single idea but rather selected from a diverse set of technically viable alternatives.


A range of concepts were explored for each major subsystem:


Drivetrain: Four-wheel drive, rear-wheel drive, and skid-steer configurations were considered to balance stability, maneuverability, and control complexity.
Navigation: IMU-based heading control, encoder-based tracking, and open-loop control were evaluated based on accuracy and implementation feasibility.
Object Collection: A scoop with rotary brush, conveyor belt system, and passive plow mechanism were explored to assess collection efficiency and object retention.
Sorting System: Both onboard sorting and a stationary base sorting system were considered, with variations in sensing and object handling mechanisms.
Homing Strategy: Ultrasonic-based positioning and alignment mechanisms were evaluated to ensure reliable return-to-base functionality.


Each concept was developed to the point of technical feasibility, ensuring that all candidate solutions were consistent with project constraints such as autonomous operation, time limitations, and system complexity.

**5. Concept Evaluation & Selection:**

The generated concepts were evaluated using structured engineering decision-making methods to determine the most effective overall system design. Three complete system-level concepts (Concept A, B, and C) were developed by combining different subsystem approaches for drivetrain, navigation, collection, and sorting. An initial Go/No-Go screening matrix was applied to eliminate concepts that failed to meet critical project requirements, such as reliable sorting performance, manageable mechanical complexity, and feasibility within the project timeline. Concepts that exhibited high risk in areas such as jamming, unreliable sorting, or excessive system complexity were rejected at this stage. The remaining concepts were then evaluated using a weighted decision matrix, where each design was assessed against key engineering criteria including collection efficiency, sorting accuracy, autonomous navigation performance, mechanical complexity, risk of failure, manufacturability, and system stability. Each criterion was assigned a relative weight based on its importance to overall system success, and concepts were scored accordingly. Through this evaluation process, Concept C achieved the highest overall score, demonstrating the best balance between performance, reliability, and implementation feasibility. Compared to alternative concepts, it reduced mechanical complexity on the mobile robot while improving sorting accuracy through the use of a stationary sorting system. Additionally, it minimized the risk of jamming and allowed for more controlled object handling during classification. The final selected concept integrates a scoop and rotary brush collection mechanism with IMU-assisted navigation and a stationary vibratory sorting system. This architecture was chosen because it enables efficient object collection while maintaining reliable and consistent sorting performance under controlled conditions, making it the most technically sound solution within the project constraints.

*TABLE 1:* Go/No-Go 

<img width="802" height="374" alt="Go-NoGo Table" src="https://github.com/user-attachments/assets/0b959545-88e5-43a3-aaa8-969707f73308" />

*TABLE 2:* Weighted decision matrix comparing Concepts A, B, and C:

<img width="810" height="271" alt="Screenshot 2026-04-07 at 11 14 16 AM" src="https://github.com/user-attachments/assets/82ee2936-8d0d-41bb-a045-9266c674ea22" />

**6. Detail Design Development:**

The selected design was refined into a fully detailed system through iterative mechanical, electrical, and control development. The final system consists of a mobile robot for object collection and a stationary base for sorting, with each subsystem designed to meet specific performance requirements. The mechanical structure was developed using CAD modeling, focusing on a lightweight 3D-printed chassis to reduce actuator load while maintaining structural stability. The robot utilizes a rear-wheel drive configuration with two DC gear motors and front support wheels to improve maneuverability and turning accuracy. The scoop mechanism was designed as a servo-actuated lifting system, allowing controlled collection and retention of objects during reverse motion. Iterative redesigns increased internal clearance and reduced interference between components, significantly improving lifting performance and reliability. The collection subsystem integrates a rotary brush mechanism driven by a DC motor, which directs objects into the scoop during forward motion. The brush rotation was refined to reverse direction during backward movement, ensuring objects remain within the scoop and minimizing losses. These refinements were based on observed failures in early prototypes and demonstrate the application of iterative design principles. The navigation and control system incorporates an IMU for heading control and ultrasonic sensing for positioning relative to the base station. A state-based control strategy (collect → return → deposit) was implemented to coordinate system behavior and ensure consistent operation. The sorting subsystem was developed as a stationary system to improve classification accuracy. It consists of a vibrating platform to regulate object flow, a color sensor for detection, and a movable chute mechanism for directing objects. Design refinements addressed issues such as object clustering and sensor misreads by improving flow control and timing, resulting in more reliable sorting performance. All major components, including motors, sensors, structural elements, and actuation systems, were selected based on performance requirements, compatibility, and feasibility within project constraints. The final design demonstrates a fully integrated system where mechanical, electrical, and control subsystems function cohesively to achieve the desired operation.

*FIGURE 1:* Annotated SolidWorks top-view assembly drawing of the final scavenger robot:


<img width="286" height="370" alt="image" src="https://github.com/user-attachments/assets/be150959-9e35-40f0-961d-bb8b12127d5a" />

*FIGURE 2:* Scoop mechanism design evolution:


<img width="378" height="135" alt="image" src="https://github.com/user-attachments/assets/cacf21c3-136b-40b9-8a56-0c7ca00fb1f8" />


*FIGURE 3:* Sorting System Final Design:


<img width="200" height="168" alt="image" src="https://github.com/user-attachments/assets/60ca37e5-460d-484a-9fbd-8996a1507ca0" />

*FIGURE 4:* System block diagram of the complete autonomous scavenger system:


<img width="316" height="231" alt="image" src="https://github.com/user-attachments/assets/e1fa9d25-a229-4a36-97cb-5aebec4080f2" />


**7. Design Analysis:**

The performance of the final design was evaluated through a combination of experimental results and engineering analysis of the system’s mechanical and sensing subsystems. The collection mechanism was assessed based on its ability to efficiently gather and retain objects. The rotary brush and scoop system demonstrated effective object capture, with brush rotation directing erasers into the scoop during forward motion. The introduction of reversed brush rotation during backward movement significantly reduced object loss. This indicates that the collection mechanism operates reliably under the expected operating conditions, with performance improvements directly resulting from iterative design refinements. The sorting subsystem was analyzed using calibrated sensor data and threshold-based classification. Experimental results showed that the normalized green value (0.4299) exceeded the detection threshold (0.40) by a margin of 0.10, providing clear separation from non-green objects. Multiple detection conditions were required to be satisfied simultaneously, and a temporal filtering method was implemented to improve robustness. As a result, no misclassification errors were observed across over 1000 tested samples, indicating high classification reliability under controlled conditions. The navigation system was evaluated based on heading stability and return-to-base performance. The use of an IMU enabled consistent orientation control, improving turning accuracy compared to open-loop methods. However, minor deviations in alignment were observed due to sensor drift and cumulative error, indicating moderate confidence in navigation precision. Overall, the system demonstrates high confidence in sorting performance, moderate confidence in navigation accuracy, and good reliability in object collection. The results confirm that the design meets the primary functional requirements, particularly in sorting accuracy and autonomous operation. Compared to alternative approaches such as onboard sorting, the selected stationary sorting system provides significantly improved sensing stability and classification accuracy. While more advanced navigation methods such as SLAM could improve positioning accuracy, they would introduce additional complexity without substantial benefit to overall system performance within the project constraints.

*FIGURE 5:* Calibration Table: The calibration data used to determine detection thresholds and validate color classification performance 


<img width="468" height="114" alt="image" src="https://github.com/user-attachments/assets/0c8c1bd7-433a-4821-802c-14c1a6817468" />

*FIGURE 6:* Integrated full-system validation results: The integrated system performance results, including sorting accuracy and overall operation time


<img width="636" height="637" alt="image" src="https://github.com/user-attachments/assets/5efbfcf1-c1c1-4e35-ae69-f1d66ad0f16f" />


The results are considered valid within the tested conditions, as consistent measurements were obtained across repeated trials under controlled environmental conditions. These findings indicate that the selected design is highly effective for sorting tasks, which is the primary objective of the system. However, the moderate confidence in navigation accuracy suggests that positional errors may affect consistency in more complex environments.


**8. Design Documentation — Mechanical:**


Detailed mechanical design documentation was developed to fully define the final system configuration. This includes CAD models, assembly drawings, and component-level designs that describe the structure, geometry, and integration of all mechanical subsystems. The mechanical design consists of a lightweight 3D-printed chassis, a servo-actuated scoop mechanism, and a rotary brush collection system, all integrated into a stable four-wheel platform. The design was developed using CAD software, allowing for accurate dimensioning, component alignment, and interference checking during the design process. The provided drawings include detailed representations of key components such as the chassis, scoop assembly, brush mounting system, and sorting interface. These drawings define critical dimensions, clearances, and assembly relationships required for proper system operation. Iterative refinements were made to improve mechanical performance, including increased clearance for the scoop mechanism and reduction of excess material to minimize weight and actuator load. The mechanical design follows standard engineering practices, including clear dimensioning, consistent part definition, and organized assembly structure. Consideration was also given to manufacturing constraints, particularly those associated with 3D printing, such as material usage, part orientation, and structural support.

The complete set of mechanical design files is available in the following repository:

- [Engineering Drawings](https://github.com/MSE2202/2026-project-pdf-team-003-1/tree/main/Final%20CADs%20and%20Drawings/Engineering%20Drawings)  
- [Scavenger Robot CAD Models](https://github.com/MSE2202/2026-project-pdf-team-003-1/tree/main/Final%20CADs%20and%20Drawings/Scavenger%20CADs)  
- [Sorting System CAD Models](https://github.com/MSE2202/2026-project-pdf-team-003-1/tree/main/Final%20CADs%20and%20Drawings/Sorter%20CADs)


**9. Design Documentation — Electrical:**

The electrical design documentation defines the complete sensing, actuation, and control architecture of the system. This includes all electronic components, wiring configurations, and signal interfaces required for the operation of both the scavenger robot and the stationary sorting system. The system integrates multiple electronic components, including DC motors for drivetrain and brush actuation, servo motors for scoop and chute movement, an IMU for heading control, ultrasonic sensors for positioning, and a color sensor for object classification. These components are coordinated through a microcontroller-based control system to enable fully autonomous operation. Detailed wiring diagrams and circuit schematics were developed to ensure proper connectivity, power distribution, and signal integrity across all subsystems. Component selection was based on compatibility, performance requirements, and ease of integration within the overall system. Special consideration was given to voltage regulation, current limits, and reliable communication between sensors and actuators. The electrical design follows standard engineering practices, including clear labeling of components, organized wiring layouts, and modular subsystem integration. Consideration was also given to practical implementation constraints such as wiring management, noise reduction, and system reliability during operation.

The complete set of electrical design files is available in the following repository:
- [Control System Code](https://github.com/MSE2202/2026-project-pdf-team-003-1/tree/main/Final%20Code%20and%20Electrical/Code)
- [Electrical References and Component Documentation](https://github.com/MSE2202/2026-project-pdf-team-003-1/tree/main/Final%20Code%20and%20Electrical/Electrical%20References)


The bill of materials (BOM) provides a complete list of all mechanical and electrical components used in the system: [Lab 003 Team 1-BOM.xlsx](https://github.com/user-attachments/files/26588690/Lab.003.Team.1-BOM.xlsx)


**10. Product Evaluation Results:**

The performance of the final system was evaluated through a series of experimental tests designed to assess its ability to meet the primary project objectives, including autonomous operation, object collection, sorting accuracy, and task completion within the required time limit. The system successfully completed the full autonomous operational cycle, demonstrating reliable integration of collection, navigation, and sorting subsystems. The system met the required time constraint and demonstrated strong sorting performance under controlled conditions. The use of a stationary sorting system contributed to improved classification stability and consistency.

The results are summarized in Figure 6, which presents the integrated system performance metrics, and in Table 6, which provides detailed validation results across multiple trials.

Testing was conducted through repeated trials under controlled conditions, with multiple erasers distributed within the arena to evaluate full system performance.

Subsystem testing confirmed that the drivetrain provided consistent motion, IMU-based heading control remained within acceptable drift limits, and ultrasonic sensing enabled reliable stopping near the base. The collection system achieved an average capture rate of 8–9 erasers per pass and maintained object retention during reverse motion. Sorting tests demonstrated accurate classification of green erasers and reliable rejection of non-target objects, with stable operation of the vibration and chute mechanisms.

Key performance results include:
- Task completion time: ~85.7 s (within 120 s requirement)
- Sorting accuracy: ~90–92%
- Misclassification: negligible under controlled conditions

While the system demonstrated strong overall performance, several limitations were identified during testing. Object clustering on the vibrating platform occasionally resulted in multiple erasers being processed simultaneously, which could affect sorting consistency. Additionally, navigation accuracy was influenced by IMU drift, leading to minor misalignment during return-to-base operations.

The results are considered reliable within the tested conditions and demonstrate strong validation of the system’s ability to meet the project objectives; however, performance may vary under different environmental conditions or increased system complexity.
