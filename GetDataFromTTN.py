# Get and plot data from TTN Console using Python

import paho.mqtt.client as mqtt
import json
import base64
#import pygmaps
from geopy.distance import geodesic
APPEUI = '******'
APPID  = '****'
PSW    = '***********'

count=0
airtime=0
def bytes_to_decimal(i,d):
    xx = i - 127
    dec = (-d if xx < 0 else d)/100
    return xx + dec

def on_connect(client, userdata, flags, rc):
    client.subscribe('+/devices/+/up'.format(APPEUI))

def on_message(client, userdata, msg):
    j_msg = json.loads(msg.payload.decode('utf-8'))
    mcounter = j_msg['counter']
    dev_eui = j_msg['hardware_serial']
    mtime=j_msg['metadata']['time']
    mfreq=j_msg['metadata']['frequency']
    mdarate=j_msg['metadata']['data_rate']
    mairtime=j_msg['metadata']['airtime']
    lat=j_msg['payload_fields']['gps_1']['latitude']
    lon=j_msg['payload_fields']['gps_1']['longitude']
    spo2=j_msg['payload_fields']['analog_3']['spo2']
    snr=j_msg['metadata']['gateways'][0]['snr']
    rssi=j_msg['metadata']['gateways'][0]['rssi']
    gateway = (40.3050998,21.7964435)
    pycom = (lat,lon)
    apostasi=geodesic(gateway, pycom).meters
    print(apostasi)
   
    print('---')
    f = open("myfile.txt", "a+")
    f.write("%d,%7.5f,%7.5f,%7.2f,%5.2f,%5.2f,%9.2f, %d\n" % (mcounter,lat,lon,apostasi,snr,rssi,mairtime,spo2))
    f.close()
    print('latidute:', lat, ' lognidute:', lon, 'snr:',snr,'rssi:',rssi,'airtime:',mairtime,spo2)
    print('dev eui: ', dev_eui)
    

print('ok')
 
ttn_client = mqtt.Client()
print('test2')
ttn_client.on_connect = on_connect
print('test3')
ttn_client.on_message = on_message
print('test4')
ttn_client.username_pw_set(APPID, PSW)
ttn_client.connect("eu.thethings.network", 1883, 60) #MQTT port over TLS

try:
    ttn_client.loop_forever()
except KeyboardInterrupt:
    print('disconnect')
    ttn_client.disconnect()