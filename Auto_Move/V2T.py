from Voice2Text import V2T
from ObjectDB import ObjectDatabase


speech2text = V2T()
db = ObjectDatabase()

def get_voice_result():
    result = speech2text.run()
    if result:
        for obj in result:
            if not db.ifExist(obj):
                return {"object" : obj, "in_memory" : False}
            else:
                row = db.selectObject(obj)
                for r in row:
                    result_dict = {
                        "object": r[1],
                        "pos" : (r[2], r[3]),
                        "delta" : (r[4], r[5]),
                        "in_memory": True
                    }
                return result_dict
    else:
        return None