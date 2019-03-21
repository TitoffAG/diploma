from pymongo import MongoClient
from datetime import datetime
from bson.binary import Binary
from pickle import dumps


def connect(url='mongodb://localhost:27017/', db_name='diploma', collection_name='results'):
    try:
        client = MongoClient(url)
        print('Connected successfully\n')
        db = client[db_name]
        return db[collection_name]
    except ConnectionError:
        print('Could not connect to MongoDB\n')


def write(collection, data, image):
    record = {
        'date': datetime.now().strftime('%d-%m-%Y/%H-%M'),
        'data': {
            'bvp_solution': data[0],
            'ivp_solution': data[1],
            'conrol': data[2],
            'functional': data[3]
        },
        'charts': Binary(dumps(image))
    }

    collection.insert(record)


def read(collection, dictionary):
    return collection.find(dictionary)
