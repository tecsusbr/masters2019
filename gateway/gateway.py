# MQTT Gateway

import paho.mqtt.client as mqtt
from threading import Thread
import serial
import json

SERVER_MQTT = "mqtt.eclipse.org"
PORT_MQTT = 1883 # 1883 = Normal Port    8883 = Encrypted port TLS 1.0 to 1.2 x509 cert
TOPIC_CMD = "master2019/strlight/cmd/"
TOPIC_REC = "master2019/strlight/rec/"
SERIAL = "COM13"

ser = serial.Serial(SERIAL, 38400)



def on_connect(con, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    con.subscribe(TOPIC_REC)

# The callback for when a PUBLISH message is received from the server.


def on_message(con, userdata, msg):
    global ser
    pay = json.loads(msg.payload)
    print(msg.topic+" "+str(pay))
    if 'uid' in pay and 'light' in pay:
        # @<UID><light>
        s = '@{:0>2}{}'.format(pay['uid'], pay['light'])
        print(s)
        ser.write(s.encode())


def sub_loop():
    sub = mqtt.Client()
    sub.on_connect = on_connect
    sub.on_message = on_message
    sub.connect(SERVER_MQTT, PORT_MQTT, 60)
    sub.loop_forever()


def pub_loop():
    pub = mqtt.Client()
    pub.connect(SERVER_MQTT, PORT_MQTT, 60)
    pub.loop_start()
    while True:
        # msg = raw_input()
        # msg = {"light": 1, "temp": 12, "hum": 41, "uid": 1}
        # pub.publish(TOPIC_CMD + "1", json.dumps(msg))
        msg = ser.readline()
        print(msg)
        # #<UID>;<light>;<temp>;<humildade>\n
        if msg:
            msg = msg.decode('utf-8')
            msg = msg[1:].split(';')
            if len(msg) == 4:
                uid, light, temp, humildade = [int(x) for x in msg]
                topico = '{}{}'.format(TOPIC_CMD, uid)
                pay = dict(light=light, temp=temp, hum=humildade, uid=uid)
                payload = json.dumps(pay)
                pub.publish(topico, payload)

sub_t = Thread(target=sub_loop, args=())
sub_t.start()

pub_t = Thread(target=pub_loop, args=())
pub_t.start()
