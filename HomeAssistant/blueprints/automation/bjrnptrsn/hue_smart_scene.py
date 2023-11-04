from datetime import datetime, timedelta
import requests

# variables
room_name = data['room']
entity_id = 'sensor.' + str(data['entity']).replace('.', '_')

def parse_config(room_name):
    global requests, datetime, timedelta

    datetimes = []
    transition = 0
    active_timeslot = 'nö'
    state = '🙄'
    
    # fetch data
    url='https://192.168.1.108/clip/v2/resource/'
    header={'hue-application-key': 'YeCctCNgXhaRGHM8B0OeRBN4UGmigFxLjFusvjjd'}
    requests.packages.urllib3.disable_warnings()
    geolocation=requests.get(url+'geolocation', headers=header, verify=False).json()
    rooms=requests.get(url+'room', headers=header, verify=False).json()
    scenes=requests.get(url+'smart_scene', headers=header, verify=False).json()
    
    # parse data to datetimes[]
    for room in rooms['data']:
        if room['metadata']['name'] == room_name:
            for scene in scenes['data']:
                if scene['group']['rid'] == room['id']:
                    name = scene['metadata']['name']
                    for timeslot in scene['week_timeslots'][0]['timeslots']:
                        if timeslot['start_time']['kind'] == 'time':
                            datetimes.append(datetime(2000, 1, 1, timeslot['start_time']['time']['hour'], timeslot['start_time']['time']['minute'], timeslot['start_time']['time']['second']))
                        if timeslot['start_time']['kind'] == 'sunset':
                            datetimes.append(datetime.strptime('2000-01-01 ' + geolocation['data'][0]['sun_today']['sunset_time'], '%Y-%m-%d %H:%M:%S'))
                    transition = scene['transition_duration']
                    active_timeslot = scene['active_timeslot']['timeslot_id']
                    state = scene['state']

    # sort datetimes by sunset
    if datetimes:
        # correct date but sunset
        for i, dt in enumerate(datetimes):
            if (i != 2) and (dt < datetimes[0]):
                datetimes[i] = dt + timedelta(days=1)

        day, evening, sunset, relax, sleep, night = datetimes
        delta = timedelta(minutes=30)

        # (sunset - 60 min) < (day)
        if sunset - (delta * 2) < day:
            # arrange (evening) and (sunset) each 30 min. after (day)
            for i in range(2):
                datetimes[i+1] = datetimes[i] + delta
        # (sunset - 30 min) < (evening)
        elif sunset - delta < evening:
            # arrange (evening) 30 min. below (sunset)
            datetimes[1] = sunset - delta
        # (sunset + 90 min) > (night)
        elif sunset + (delta * 3) > night:
            # arrange (sleep, relax and sunset) below (night) at intervals of 30 min. each
            for i in range(5, 2, -1):
                datetimes[i-1] = datetimes[i] - delta
        # (sunset + 30 min) > (relax)  
        elif sunset + delta > relax:
            # arrange (relax, sleep) each at least 30 min. after (sunset)
            for i in range(2, 4):
                datetimes[i+1] = max(datetimes[i] + delta, datetimes[i+1])
    else:
        for i in range(6):
            datetimes.append(datetime(2000, 1, 1))

    return name, datetimes, transition, active_timeslot, state

def send_config(hass, room_name, entity_id, config):
    global datetime
    time_format = '%H:%M:%S'
    name, (day, evening, sunset, relax, sleep, night), transition, timeslot, state = config
    service_data = {
        "room": room_name,
        "name": name,
        "day": datetime.strftime(day, time_format),
        "evening": datetime.strftime(evening, time_format),
        "sunset": datetime.strftime(sunset, time_format),
        "relax": datetime.strftime(relax, time_format),
        "sleep": datetime.strftime(sleep, time_format),
        "night": datetime.strftime(night, time_format),
        "transition": transition,
        "timeslot": timeslot,
        "entity": entity_id
    }

    hass.states.set(entity_id, state, service_data)

send_config(hass, room_name, entity_id, parse_config(room_name))
