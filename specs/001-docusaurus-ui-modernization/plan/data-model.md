# Data Model: Docusaurus UI/UX Components

**Feature**: Modern Docusaurus UI for Physical AI and Humanoid Robotics Textbook
**Created**: 2025-12-26
**Status**: Complete

## Entity Models

### Module
- **Description**: Major section of the textbook containing multiple chapters
- **Fields**:
  - id: unique identifier for the module
  - title: display name of the module (e.g., "Module 1: The Robotic Nervous System")
  - position: ordering index for navigation
  - chapters: collection of chapters within the module
  - description: brief summary of module content
  - icon: optional visual identifier for the module card

### Chapter
- **Description**: Subsection within a module focusing on specific concepts and topics
- **Fields**:
  - id: unique identifier for the chapter
  - title: display name of the chapter
  - module_id: reference to parent module
  - position: ordering index within the module
  - topics: collection of topics within the chapter
  - description: brief summary of chapter content
  - estimated_reading_time: time in minutes to complete the chapter
  - prerequisites: list of required knowledge or previous chapters

### Topic
- **Description**: Individual sections within chapters covering specific aspects of the material
- **Fields**:
  - id: unique identifier for the topic
  - title: display name of the topic
  - chapter_id: reference to parent chapter
  - position: ordering index within the chapter
  - content: reference to the actual content document
  - tags: list of relevant topics or concepts covered
  - difficulty_level: indicator of complexity (beginner, intermediate, advanced)

### NavigationItem
- **Description**: Individual item in the navigation hierarchy
- **Fields**:
  - id: unique identifier for the navigation item
  - label: display text for the navigation item
  - type: indicates if it's a module, chapter, or topic
  - href: URL path to the content
  - children: collection of sub-items in the navigation
  - is_active: boolean indicating if this is the current page
  - collapsible: boolean indicating if this section can be collapsed

### ThemePreference
- **Description**: User's theme selection preference
- **Fields**:
  - user_id: identifier for the user (or session)
  - theme: selected theme (light, dark, or system)
  - last_updated: timestamp of last preference change
  - persistence_method: how the preference is stored (localStorage, cookie, etc.)

### ReadingProgress
- **Description**: User's progress through content
- **Fields**:
  - user_id: identifier for the user
  - content_id: reference to the content being read
  - progress_percentage: percentage of content completed
  - time_spent: total time spent reading this content
  - last_accessed: timestamp of last reading session
  - completed: boolean indicating if the content was completed

### EducationalComponent
- **Description**: Custom UI components for educational purposes
- **Fields**:
  - type: category of component (Note, Tip, Warning, Example, Definition)
  - title: optional header for the component
  - content: main content of the educational component
  - icon: visual indicator for the component type
  - collapsible: boolean indicating if the component can be collapsed

### PageMetadata
- **Description**: Metadata for content pages to support reading aids
- **Fields**:
  - page_id: reference to the content page
  - estimated_reading_time: calculated time to read the page
  - word_count: total words in the content
  - reading_level: complexity level of the content
  - related_pages: list of related content for navigation
  - breadcrumbs: navigation path to this page

## Validation Rules

### Module Validation
- Title must be non-empty and descriptive
- Position must be unique within the textbook
- Must contain at least one chapter
- ID must be unique across all modules

### Chapter Validation
- Title must be non-empty and descriptive
- Position must be unique within the parent module
- Module_id must reference an existing module
- Estimated reading time must be positive

### Topic Validation
- Title must be non-empty and descriptive
- Position must be unique within the parent chapter
- Chapter_id must reference an existing chapter
- Difficulty level must be one of: beginner, intermediate, advanced

### NavigationItem Validation
- Label must be non-empty
- Type must be one of: module, chapter, topic
- Href must be a valid relative path
- If collapsible, must have children

### ThemePreference Validation
- Theme must be one of: light, dark, system
- User_id must be valid for the persistence method

### ReadingProgress Validation
- Progress percentage must be between 0 and 100
- Time spent must be non-negative
- Content_id must reference existing content

### EducationalComponent Validation
- Type must be one of: Note, Tip, Warning, Example, Definition
- Content must be non-empty
- If collapsible, content should be substantial enough to justify the feature

## State Transitions

### ThemePreference States
- Initial: system default
- Changed: user selects a specific theme
- Updated: theme preference is stored and applied

### ReadingProgress States
- Started: user begins reading content
- In Progress: user has read some portion of the content
- Completed: user has finished reading the content
- Reset: user resets progress

## Relationships

- Module → Chapter: One-to-many (one module contains many chapters)
- Chapter → Topic: One-to-many (one chapter contains many topics)
- NavigationItem → NavigationItem: Self-referencing hierarchy (parent-child relationships)
- ThemePreference → User: Many-to-one (many preferences for one user context)
- ReadingProgress → Content: Many-to-one (many progress records for one content item)
- PageMetadata → Content: One-to-one (metadata for each content page)

## Indexes

- Module.position: For efficient ordering in navigation
- Chapter.module_id: For efficient module-to-chapter queries
- Topic.chapter_id: For efficient chapter-to-topic queries
- NavigationItem.type: For efficient navigation rendering
- ReadingProgress.user_id: For efficient user progress retrieval
- PageMetadata.page_id: For efficient metadata access