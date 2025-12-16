# Data Model: Module 1 - The Robotic Nervous System (ROS 2)

## Chapter Content Entity

**Description**: Represents the structured educational material for each of the 3 chapters, including learning objectives, explanations, code examples, labs, summaries, and exercises

**Fields**:
- `chapterId`: String (e.g., "ch1-intro-to-ros2")
- `title`: String (e.g., "Introduction to ROS 2 Architecture")
- `learningObjectives`: Array<String> (3-5 learning objectives)
- `contentSections`: Array<ContentSection>
- `codeExamples`: Array<CodeExample>
- `labExercises`: Array<LabExercise>
- `summary`: String (chapter summary)
- `exercises`: Array<Exercise>
- `references`: Array<Reference>
- `assets`: Array<String> (file paths to diagrams, images)

**Validation Rules**:
- `chapterId` must match the pattern: ch[1-9]-[a-z0-9-]+
- `learningObjectives` must contain 3-5 items
- `references` must contain at least 5 items with 40% being peer-reviewed
- `codeExamples` must be ROS 2 Humble compatible

## Content Section Entity

**Description**: Represents individual sections within a chapter that explain concepts

**Fields**:
- `sectionId`: String (unique identifier)
- `title`: String (section title)
- `content`: String (explanatory text)
- `diagrams`: Array<String> (asset file paths)
- `examples`: Array<String> (code example IDs)

**Validation Rules**:
- `content` must be accessible to undergraduate learners
- `diagrams` must enhance understanding of the concept

## Code Example Entity

**Description**: Represents practical implementation examples using ROS 2 rclpy API that students can follow and execute

**Fields**:
- `exampleId`: String (unique identifier)
- `language`: String (e.g., "python")
- `code`: String (the actual code content)
- `description`: String (what the example demonstrates)
- `dependencies`: Array<String> (ROS packages required)
- `expectedOutput`: String (what the example should produce)

**Validation Rules**:
- `code` must be clean, minimal, and well-commented
- `code` must be compatible with ROS 2 Humble
- `expectedOutput` must be clearly defined

## Lab Exercise Entity

**Description**: Represents the hands-on practical activities that reinforce theoretical concepts through simulation

**Fields**:
- `exerciseId`: String (unique identifier)
- `title`: String (exercise title)
- `objectives`: Array<String> (what students will learn)
- `prerequisites`: Array<String> (what students need to know)
- `steps`: Array<LabStep>
- `expectedResult`: String (what students should achieve)
- `simulationEnvironment`: String (e.g., "Gazebo")

**Validation Rules**:
- `simulationEnvironment` must be accessible without hardware dependencies
- `steps` must be clearly defined and reproducible

## Lab Step Entity

**Description**: Represents individual steps within a lab exercise

**Fields**:
- `stepNumber`: Integer (sequential numbering)
- `description`: String (what to do)
- `code`: String (code to write, if applicable)
- `verification`: String (how to verify success)

**Validation Rules**:
- `stepNumber` must be sequential starting from 1
- `verification` must be objective and measurable

## Reference Entity

**Description**: Represents academic or technical references in IEEE format

**Fields**:
- `referenceId`: String (unique identifier)
- `type`: String (e.g., "academic-paper", "official-documentation", "textbook")
- `ieeeCitation`: String (properly formatted IEEE citation)
- `url`: String (optional, if available online)
- `accessDate`: Date (when the reference was accessed)

**Validation Rules**:
- `ieeeCitation` must follow IEEE citation style
- At least 40% of references in a chapter must be peer-reviewed

## State Transitions

### Chapter Content Lifecycle
1. **Draft**: Initial creation with basic structure
2. **Research Complete**: All technical concepts validated
3. **Content Developed**: All sections written with examples
4. **Lab Integrated**: Simulation exercises added
5. **Reviewed**: Peer review completed
6. **Published**: Ready for textbook inclusion

## Relationships

- **Chapter Content** 1 → * **Content Section**: One chapter contains multiple sections
- **Chapter Content** 1 → * **Code Example**: One chapter contains multiple code examples
- **Chapter Content** 1 → * **Lab Exercise**: One chapter contains multiple lab exercises
- **Chapter Content** 1 → * **Reference**: One chapter references multiple sources
- **Lab Exercise** 1 → * **Lab Step**: One lab exercise contains multiple steps
- **Content Section** * → * **Code Example**: Sections may reference multiple code examples