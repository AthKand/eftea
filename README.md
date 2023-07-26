# eftea

(Tiny) F-T and point-of-application tester for the FT 300-s and the mTMS 

## Installation 
eftea assumes you have the FT 300-s driver sensor package install and can run it on your host machine. Download [here](https://assets.robotiq.com/website-assets/support_documents/document/robotiq_ft_sensor_dev_v1.0.1_20210317.zip?_ga=2.176475628.874552999.1690361216-1202197697.16886464880). 

Follow the required installation steps on the website. 

Create an environment and run the required install script 

```bash
pip install -r requirements.txt
```

## Usage
1. Run ./driverSensor script from the robotiq sensor development package
2. Wait to receive a reading from the sensor (takes roughly 3-5 seconds)
3. Run the script 

```cli
python ft.py 
```

### Features
1. Live plotting of force and moment values from the sensor.
2. Grpahical depiction of 2-D point of application of force (on the face). you may change shape from within the code.  

## Contributing

Pull requests are welcome.


