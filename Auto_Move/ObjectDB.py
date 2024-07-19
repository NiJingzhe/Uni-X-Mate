import numpy as np
import psycopg2
import pickle


class ObjectDatabase:
    def __init__(self, host='172.25.106.21', database='postgres', user='postgres', password='123456'):
        self.conn = psycopg2.connect(
            host=host,  # 数据库服务器地址
            database=database,  # 数据库名
            user=user,  # 数据库用户名
            password=password  # 数据库用户密码
        )

    def createDB(self):
        try:
            cur = self.conn.cursor()
            create_table_query = '''
            CREATE TABLE IF NOT EXISTS objects (
                id SERIAL PRIMARY KEY,
                name VARCHAR(255),
                x FLOAT,
                y FLOAT,
                delta_x FLOAT,
                delta_y FLOAT,
                image BYTEA
            )
            '''
            cur.execute(create_table_query)
            self.conn.commit()
            print('Successfully create the object database')
        except Exception as e:
            self.conn.rollback()
            print(e)

    def deleteDB(self):
        try:
            cur = self.conn.cursor()
            cur.execute('''
                drop table objects
            ''')
            self.conn.commit()
            print('Successfully drop the object database')
        except Exception as e:
            self.conn.rollback()
            print(e)

    def insertObject(self, name, x, y, delta_x, delta_y, image):
        try:
            cur = self.conn.cursor()
            cur.execute(
                """
                insert into objects (name, x, y, delta_x, delta_y, image) 
                values (%s, %s, %s, %s, %s, %s)
                """, (name, x, y, delta_x, delta_y, pickle.dumps(image))
            )
            self.conn.commit()
            return True
        except Exception as error:
            print(error)
            self.conn.rollback()
            return False

    def selectObject(self, name):
        try:
            cur = self.conn.cursor()
            cur.execute(
                """
                select * from objects where name = %s
                """, (name,)
            )
            result = cur.fetchall()
            return result
        except Exception as error:
            print(error)
            return []

    def modifyObject(self, name, x, y, delta_x, delta_y, image):
        try:
            cur = self.conn.cursor()
            cur.execute(
                """
                update objects set x = %s, y = %s, delta_x = %s, delta_y = %s, image = %s where name = %s
                """, (x, y, delta_x, delta_y, pickle.dumps(image), name)
            )
            self.conn.commit()
            return True
        except Exception as error:
            print(error)
            self.conn.rollback()
            return False

    def ifExist(self, name):
        try:
            cur = self.conn.cursor()
            cur.execute(
                """
                select id from objects where name = %s
                """, (name,)
            )
            result = cur.fetchone()
            return result is not None
        except Exception as error:
            print(error)
            return False


if __name__ == '__main__':
    db = ObjectDatabase()
    # db.createDB()
    # db.deleteDB()
