DRON_SIM/
│
├── .devcontainer/
│   └── devcontainer.json     # Konfiguracja środowiska programistycznego (Docker + VSCode)
│
├── .vscode/
│   ├── c_cpp_properties.json # ustawienia kompilatora C++
│   ├── settings.json         # ustawienia VSCode
│   └── tasks.json            # automatyczne zadania np. budowanie, uruchamianie
│
├── build/                    #|
├── install/                  #| -> artefakty ROS2 // Tworzone automatycznie przez "colcon build"
├── log/                      #|
│
├── sim_bringup/              # pakiet ROS2 w pythonie
│
├── scripts/                  # miejsce na skrypty symulacyjne
│
├── Dockerfile                # definicja obrazu kontenera (można porównać do programowania obiektowego gdzie: obraz = instancja, kontener = klasa)
├── package.xml               # deklaracja pakietu ROS2
├── pyproject.toml            # konfiguracja środowiska pythona
├── README.md                 # instrukcja korzystania z oprogramowania
├── requirements.txt          # zależności Pythona instalowane w środowisku
└── .gitignore                # pliki ignorowane przez gita


DRON_SIM/
|
|--sim_brinup           # rozłożenie pakietu ROS2 na części
|   |
|   |--build            # pliki kompilacji (tymczasowe artefakty)
    |--install          # zainstalowane pliki wynikowe, które ROS2 faktycznie ładuje
    |--launch           # zawiera pliki uruchamiające symulacje (pojedyńczy launch uruchamia wiele nodes)
    |--models           # zawiera modele wykorzystane w symulacji
    |--resource         # zawiera metadane pakietu - zwykle plik o tej samej nazwie co pakiet
    |--worlds           # zawiera pliki świata Gazebo (.sdf), które definiują, środowisko symulacji, pozycje startową drona, fizyke i czas symulacji
    |--package.xml      # to główny plik metadanych pakiety ROS2
    |--setup.cfg        # plik konfiguracyjny dla setuptools, ułatwia ROS2 znalezienie skryptów uruchamialnych
    |--setup.py         # plik instalacyjny Pythona. Definiuje, jak ROS2 ma zainstalować Twój pakiet i jakie pliki mają trafić do install/
                          to serce instalacji pakietu - mówi ROS2, które katalogi i pliki mają być dostępne po "colcon build"

ros_gz_bridge to pakiet który "tłumaczy" wiadomość (messages) między dwoma systemami

Przykład: Creating GZ->ROS Bridge: [/clock (gz.msgs.Clock) -> /clock (rosgraph_msgs/msg/Clock)] (Lazy 0)

- Gazebo generuje własny czas symulacji (gz.msgs.Clock)
- Bridge publikuje go do ROS-a na /clock (typ rosgraph_msgs/msg/Clock)
- Dzięki temu ROS wie, jaki jest aktualny czas symulacji (nie rzeczywisty systemowy)
#(Lazy 0) oznacza, że bridge nie używa trybu "lazy", czyli zawsze aktywnie przekazuje dane, nawet jeśli w ROS-ie nikt jeszcze nie subskrybuje danego tematu.
#(Lazy 1) oznacza, że bridge dopiero tworzy połączenie, gdy pojawi się subskrybent.

Potrzebne do synchronizacji wszystkich ROS-nodów w środowisku symulacyjnym



Założenie projektowe:

DRON_SIM/
├── src/
    ├── uav_bringup/                # tylko launch + zasoby świata (worlds, models)
    ├── uav_sim/                 # mapy/tereny, modele ludzi/przeszkód (opcjonalnie osobno od bringup)
    ├── uav_interfaces/             # (opcjonalnie) własne msg/srv, jeśli nie wystarczą standardy
    ├── uav_sensors/                # konfiguracja/mostki kamer, synchronizacja, rectify
    ├── uav_depth/                  # estymacja głębi (mono/stereo), tworzenie depth/pointcloud
    ├── sar_obstacle_avoidance/     # unikanie przeszkód (local planner + cmd_vel)
    ├── sar_people_detection/       # detekcja osób (YOLO/etc.), publikacja detekcji
    ├── sar_mission/                # logika misji/tryby pracy (search patterns, loiter, failsafe)
    ├── sar_eval/                   # logowanie, metryki, scenariusze testowe, raporty
    └── sar_viz/                    # RViz, dashboard, rqt, overlay HUD (opcjonalnie)

Overlay to sposób, w jaki ROS 2 pozwala Ci mieć kilka workspace'ów(środowisk), które nakładają się na siebie (ang. overlaying workspaces).


Zainstalowanie zależności do geotag_recordera:

# 1) Sprawdź, do którego Pythona trafiają pakiety
which python3
python3 -c "import sys; print(sys.executable)"
python3 -m pip --version

# 2) Usuń pip-owe pakiety, które nadpisują system
python3 -m pip uninstall -y numpy opencv-python opencv-python-headless opencv-contrib-python opencv-contrib-python-headless

# 3) Zainstaluj wersje zgodne z ROS 2 Humble
python3 -m pip install "numpy==1.26.4" "opencv-python<4.10" piexif


