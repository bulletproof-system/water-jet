#!/usr/bin/env python
# coding=utf-8
import rospy
import sqlite3
from datetime import datetime
from pot_database.msg import PotInfo, PotUpdate
from pot_database.srv import *
import pickle

class Database:
    def __init__(self):
        rospy.init_node('database')

        # TODO DATABASE PATH
        self.db_path = './pots.db'
        self.connect_to_database()

        # Publishers
        self.update_pub = rospy.Publisher('/database/pot/update', PotUpdate, queue_size=10)

        # Services
        rospy.Service('/database/pot/list', GetPotList, self.handle_pot_list)
        rospy.Service('/database/pot/set', SetPotInfo, self.handle_set_pot_info)
        rospy.Service('/database/pot/set_active', SetPotActive, self.handle_set_pot_active)
        rospy.Service('/database/pot/get', GetPotInfo, self.handle_get_pot_info)
        rospy.Service('/database/pot/delete', DeletePot, self.handle_delete_pot)

    def connect_to_database(self):
        try:
            self.conn = sqlite3.connect(self.db_path, check_same_thread=False)
            self.cursor = self.conn.cursor()
            self.setup_database()
        except sqlite3.Error as e:
            rospy.logerr("Database connection failed: %s" % str(e))
            raise

    def setup_database(self):
        try:
            self.cursor.execute('''
                CREATE TABLE IF NOT EXISTS pots (
                    id INTEGER PRIMARY KEY,
                    pose BLOB,
                    data BLOB,
                    picture BLOB,
                    active BOOLEAN,
                    last_water_date DATETIME
                )
            ''')
            self.conn.commit()
        except sqlite3.Error as e:
            rospy.logerr("Failed to setup database: %s" % str(e))
            raise

    def handle_pot_list(self, request):
        try:
            self.cursor.execute('SELECT * FROM pots')
            rows = self.cursor.fetchall()
            pots = [self.row_to_potinfo(row) for row in rows]
            return GetPotListResponse(pots=pots)
        except sqlite3.Error as e:
            rospy.logerr("Failed to retrieve pot list: %s" % str(e))
            return GetPotListResponse(pots=[])

    def handle_set_pot_info(self, request):
        pot = request.info
        try:
            if pot.last_water_date == "":
                last_water_date = None
            else:
                last_water_date = datetime.strptime(pot.last_water_date, "%Y-%m-%dT%H:%M:%S")

            # Serialize pose, data, and picture using pickle
            serialized_pose = pickle.dumps(pot.pose)
            serialized_data = pickle.dumps(pot.data)
            serialized_picture = pickle.dumps(pot.picture)            

            self.cursor.execute('''
                INSERT OR REPLACE INTO pots (id, pose, data, picture, active, last_water_date)
                VALUES (?, ?, ?, ?, ?, ?)
            ''', (pot.id, serialized_pose, serialized_data, serialized_picture, pot.active, last_water_date))
            self.conn.commit()
            self.publish_update([pot.id], [])
            return SetPotInfoResponse(success=True)
    
        except sqlite3.Error as e:
            rospy.logerr("Failed to set pot info, DB error: %s" % str(e))
            self.conn.rollback()
            return SetPotInfoResponse(success=False)
        except ValueError as ve:
            rospy.logerr("Failed to parse last watering date: %s" % str(ve))
            return SetPotInfoResponse(success=False)

    def handle_set_pot_active(self, request):
        try:
            self.cursor.execute('''
                UPDATE pots SET active = ? WHERE id = ?
            ''', (request.active, request.id))
            self.conn.commit()
            self.publish_update([request.id], [])
            return SetPotActiveResponse(success=True)
        except sqlite3.Error as e:
            rospy.logerr("Failed to set pot active: %s" % str(e))
            self.conn.rollback()
            return SetPotActiveResponse(success=False)

    def handle_get_pot_info(self, request):
        try:
            self.cursor.execute('SELECT * FROM pots WHERE id = ?', (request.id,))
            row = self.cursor.fetchone()
            if row:
                return GetPotInfoResponse(success=True, info=self.row_to_potinfo(row))
            return GetPotInfoResponse(success=False)
        except sqlite3.Error as e:
            rospy.logerr("Failed to get pot info: %s" % str(e))
            return GetPotInfoResponse(success=False)

    def handle_delete_pot(self, request):
        try:
            self.cursor.execute('DELETE FROM pots WHERE id = ?', (request.id,))
            self.conn.commit()
            self.publish_update([], [request.id])
            return DeletePotResponse(success=True)
        except sqlite3.Error as e:
            rospy.logerr("Failed to delete pot: %s" % str(e))
            self.conn.rollback()
            return DeletePotResponse(success=False)

    def publish_update(self, update_list, delete_list):
        update_msg = PotUpdate(update_list=update_list, delete_list=delete_list)
        self.update_pub.publish(update_msg)

    def row_to_potinfo(self, row):
        last_water_date_str = row[5].strftime("%Y-%m-%dT%H:%M:%S") if row[5] else None
        return PotInfo(id=row[0], pose=row[1], data=row[2], picture=row[3], 
                       active=row[4], last_water_date=last_water_date_str)

if __name__ == '__main__':
    try:
        pot_db = Database()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("Unhandled exception: %s" % str(e))
