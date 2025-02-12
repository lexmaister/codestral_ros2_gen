# Codestral ROS2 AI Generator
Generate ROS2 elements (nodes, interfaces, etc) with Codestral AI model

## Generation block diagram

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
        Timer --> GenCode[Generate Service Code]
        GenCode --> SaveCode[Save to service_node file]
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

[Mermais flowchart options](https://mermaid.js.org/config/schema-docs/config-defs-flowchart-diagram-config.html#flowchartdiagramconfig-properties)

