# BAVCAV_PROJECT
ROS package that enables full visual and vocal control of the DJI Tello. New commands coming soon...


# POCKETSPHINX INSTALLATION
sudo pip install SpeechRecognition  
sudo apt-get install python-pyaudio python3-pyaudio  
python3 -m pip install --upgrade pip setuptools wheel  
pip install --upgrade pocketsphinx  

# If you get the error “failed building wheel”
Try: sudo apt-get install pulseaudio swig libpulse-dev  
If it still gives error try: sudo apt-get install -qq python python-dev python-pip build-essential swig git libpulse-dev libasound2-dev  
If error remains: specify python3 wherever python is mentioned  


# DIRECTORY CHANGES
To use vocal commands and take captures, directory variables MUST BE CHANGED  
In sensor_processing.py, look for declaration of "lm" and "dic" variables and set the directory to the absolute path of your words.lm and words.dic files  
In driver.py, look for declaration of the "current_path" variable and set it to the absolute directory of the project folder  


# USING COMMANDS
For each desired command, preface your phrase with the keyword "COMMAND" via vocal input  
This applies to both vocal and visual commands  


# FURTHER DOCUMENTATION + GESTURE TABLE
https://docs.google.com/document/d/1emxRx9QORcGIwwhwwRmZfzS8k41XA_UvCmYcOlu-Nvo/edit?usp=sharing  


# VOCAL COMMAND DOCUMENTATION
A list of vocal commands can be found in speech/words.txt  


