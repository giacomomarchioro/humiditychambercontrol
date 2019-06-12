import sqlalchemy as db
import serial
from datetime import datetime

# Create the database or access it.
engine = db.create_engine('sqlite:///CClog.sqlite')
connection = engine.connect()
metadata = db.MetaData()

emp = db.Table('emp', metadata,
               db.Column('Motor_au', db.Integer()),
               db.Column('T_BP280_DegC', db.Float()),
               db.Column('P_BP280_hPa', db.Float()),
               db.Column('T_DHT11_DegC', db.Float()),
               db.Column('HR_DHT11_percent', db.Float()),
               db.Column('Time_stamp', db.DateTime())
               )

metadata.create_all(engine) #  Creates the table
# Serial interface.
serial_port = '/dev/ttyACM0'
baud_rate = 9600


with serial.Serial(serial_port, baud_rate) as ser:
    ser.flushInput()  # maybe should be also output
    print("Start reading from: %s" % (ser.name))
    while True:
        try:
            ser_line = ser.readline()
            # We check if the packet is complete S at the begining
            # and E a the end [-3] (E\n).
            if ser_line[0] == 'S' and ser_line[-3] == 'E':
                values = ser_line.split(';')
                query = db.insert(emp).values(Motor_au=values[1],
                                              T_BP280_DegC=values[2],
                                              P_BP280_hPa=values[3],
                                              T_DHT11_DegC=values[4],
                                              HR_DHT11_percent=values[5],
                                              Time_stamp=datetime.utcnow())
                ResultProxy = connection.execute(query)
            else:
                print('Missed reading at %s' % (datetime.utcnow()))
                print('Read: %s' % (ser_line))

        except:
            print("Keyboard Interrupt")
            break
