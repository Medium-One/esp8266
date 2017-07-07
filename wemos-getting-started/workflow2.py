import M1Geolocation
import Store
import json

# get IP from event
ip_address = IONode.get_input('in1')['event_data']['value']

try:
    geo_data = M1Geolocation.get_location_from_ip(ip_address)
    IONode.set_output('out1', geo_data)
    # update device gps data
    geo_point  = str(geo_data['location']['latitude']) + " " + str(geo_data['location']['longitude'])
    # save gps data separately to be used for the maps
    IONode.set_output('out2', {'gps':geo_point})
except Exception:
    log("Failed to get IP lookup")

