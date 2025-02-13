# Codestral ROS2 AI Generator
Generate ROS2 elements (nodes, interfaces, etc) with Codestral AI model

## Installation

* Make a directory for the project and navigate to it:
```bash
mkdir test_gen
cd test_gen
```

* Create a virtual environment:
```bash
python3 -m venv .venv
source .venv/bin/activate
```

* Clone the repository:
```bash
git clone https://github.com/lexmaister/codestral_ros2_gen.git
cd codestral_ros2_gen
```

* Install the package in the editable mode:
```bash
pip install --upgrade pip
pip install -e .
```

## Usage

To use this project, you need to have ROS2 installed (it was designed for ROS2 Humble). Please refer to the [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation.html) for instructions.

### Examples

To use examples, you also need to create a directory for ros2 workspace `ros2_ws` **near your project directory**:
```bash
mkdir test_ws
```

so your directory structure will look like this:

```
test_gen/
├── .venv/
├── codestral_ros2_gen/
└── test_ws/
```

* [Generating a simple service node](docs/Example_object_height_service.md)

## Generation and Performance Evaluation Block Diagram

```mermaid
%%{ 
init: { 
     'theme': 'base',
    'themeVariables': {
      'primaryColor': '#BB2528',
      'primaryTextColor': '#000',
      'primaryBorderColor': '#7C0000',
      'secondaryColor': '#E9967A',
      'tertiaryColor': '#fff'
    },
    'flowchart': {
        'useMaxWidth': true,
        'curve': 'basis',
        'subGraphTitleMargin':{
            "top": 5,
            "bottom": 20
            },
        'nodeSpacing': 25, 
        'rankSpacing': 25,
        'padding': 0,
        'wrappingWidth':1200,
        'defaultRenderer': 'dagre-wrapper'
        } 
    } 

}%%
flowchart TD
    classDef toolbox fill:#2f4f4f,stroke:#fff,stroke-width:2px,color:#fff,font-size:13pt
    classDef default fill:#fff5e6,stroke:#666,color:#333,font-size:13pt
    classDef diamond fill:#fff5e6,stroke:#666,color:#333,font-size:13pt

    Start([Start]) ---> MainTimer

    subgraph Generation[<span style='font-size:20px; font-weight:bold; display:block; padding-right:250px; '>Generation Phase</span>]
        direction TB       

        MainTimer[Initialize Main Timer] --> Counter[Initialize Attempt Counter]
        Counter --> Timer[Start Attempt Timer]
        Timer --> GenCode[Generate Element Code]
        GenCode --> SaveCode[Save to file inside ros2 package]
        SaveCode --> RunTests[Execute Tests]
        RunTests --> TestResult{Tests Passed?}:::diamond
        TestResult -->|No| RecordFail[Record Failed Attempt]
        TestResult -->|Yes| RecordSuccess[Record Success]
        RecordFail --> Counter
    end

    RecordSuccess --> StoreResults[Store Iteration Results]
    StoreResults --> IterationDone{30 Iterations Done?}:::diamond
    IterationDone -->|No| MainTimer
    IterationDone -->|Yes| Report[Generate Performance Report]
    Report --> End([End])

    T1[Mistral AI]:::toolbox
    T2[File I/O]:::toolbox
    T3[pytest]:::toolbox
    T4[ROS2]:::toolbox
    T5[Pandas]:::toolbox

    GenCode -.->|uses| T1
    SaveCode -.->|uses| T2
    RunTests -.->|uses| T3
    RunTests -.->|uses| T4
    Report -.->|uses| T5

    linkStyle default stroke:#666,stroke-width:3px,font-size:12pt
    linkStyle 10,11,12,13,14 stroke:#2f4f4f,stroke-width:2px,stroke-dasharray:5,font-size:13pt
```

[Mermaid flowchart options and customization](https://mermaid.js.org/config/schema-docs/config-defs-flowchart-diagram-config.html#flowchartdiagramconfig-properties)

