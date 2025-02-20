#!/usr/bin/env python
#
# Example of handling a csv file as a database the example is books and authors
# it then shows how to load that data into a sqllite3 database table
#
import sqlite3
import csv
import os
from os import path
from dataclasses import dataclass
from enum import Enum
from glob import glob
from pprint import pprint
DATA_DIR = path.join(path.dirname(__file__), "data")

# define the record structure, this is the format of each row in the csv records
#
@dataclass
class csvFileItem():
    author_id: str
    author: str
    title_id: str
    title: str
    category_type: str
    file_path: str
    file_size: int

# hansle the csv database with search etc...
#
class csvDB():
    TYPE_TO_SEARCH = "Type2FilterBy"

    def __init__(self, data_dir=DATA_DIR, datdb='book_list.db'):
        self.data_dir = data_dir
        self.csv_file = path.join(data_dir, "list_person_all.csv")
        self.card_dir = path.join(data_dir, "cards")
        self.load()
        self.dbname = datdb
        self.conn = sqlite3.connect(self.dbname)
        self.cur = self.conn.cursor()
        self.make_sql_table()
        
    def load(self, filter_type_sch=False):

        self.data = []
        with open(self.csv_file, mode="r", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                if filter_type_sch and row["category_type"] != csvDB.TYPE_TO_SEARCH:
                    continue
                file_path = self.find_file_path(row["author_id"], row["title_id"])
                if file_path:
                    item = csvFileItem(author_id=row["author_id"],
                                      author=row["author"],
                                      title_id=row["title_id"],
                                      title=row["title"],
                                      category_type=row["category_type"],
                                      file_path=file_path,
                                      file_size=path.getsize(file_path))
                    self.data.append(item)
                else:
                    pass

    def find_file_path(self, author_id, title_id):
        author_dir = path.join(self.card_dir, author_id, "files")
        if not path.isdir(author_dir):
            return None
        title_id = str(int(title_id, 10))                                                   # remove zero padding
        text_dirs = [path.join(author_dir, ent) for ent in os.listdir(author_dir) if ent.startswith(title_id + "_")]
        if not text_dirs:
            return None
        if len(text_dirs) > 1:
            def get_update_id(file_path):
                basename = path.splitext(path.basename(file_path))[0]
                key = basename.split("_")[-1]
                if key == "txt":
                    update_id = 0
                elif key == "ruby":
                    update_id = 1
                else:
                    update_id = int(key) if key.isdigit() else 0
                return update_id
            tmp = [(f, get_update_id(f)) for f in text_dirs]
            tmp = sorted(tmp, key=lambda l: l[1], reverse=True)
            text_dirs = [l[0] for l in tmp]
        texts = [path.join(text_dirs[0], text) for text in os.listdir(text_dirs[0])]
        if len(texts) == 0:
            print("warning: no text", text_dirs[0], file=sys.stderr)
            return None
        elif len(texts) > 1:
            print("warning multi text", texts, file=sys.stderr)
        return texts[0]

    @staticmethod
    def filter_search(items):
        return [item for item in items if item.category_type == csvDB.TYPE_TO_SEARCH]

    @staticmethod
    def order_by_size(items):
        return list(sorted(items, key=lambda item: item.file_size, reverse=True))

    def find_by_title(self, title=None, keyword=None):
        if title is not None:
            return [item for item in self.data if title == item.title]
        elif keyword is not None:
            return [item for item in self.data if keyword in item.title]
        else:
            raise ValueError("title or keyword must be specified")

    def add_title_to_db(self, title=None):
        if title is not None:
            its = item for item in self.data if title == item.title]
            for each_line in its:
                self.add_sql_table(each_line.author, each_line.title, each_line.file_size)
        else:
            raise ValueError("title or keyword must be specified")

    def add_author_to_db(self, author=None):
        if author is not None:
            its = item for item in self.data if author == item.author]
            for each_line in its:
                self.add_sql_table(each_line.author, each_line.title, each_line.file_size)
        else:
            raise ValueError("title or keyword must be specified")

    def add_csv_db(self, author=None):
        for each_line in self.data:
            self.add_sql_table(each_line.author, each_line.title, each_line.file_size)
            
    def find_by_author(self, author=None, keyword=None, filter_type_sch=True, size_order=True, limit=None, size_max=0, size_min=0):
        if author is not None:
            items = [item for item in self.data if author == item.author]
        elif keyword is not None:
            items = [item for item in self.data if keyword in item.author]
        else:
            raise ValueError("author or keyword must be specified")
        if filter_type_sch:
            items = self.filter_search(items)
        if size_order:
            items = self.order_by_size(items)
        if size_max > 0:
           items = [i for i in items if size_max > i.file_size]      
        if size_min > 0:
           items = [i for i in items if size_min < i.file_size]            
        if isinstance(limit, int):
           items = items[:limit]
        return items

    def __len__(self):
        return len(self.data)

    def make_sql_table(self):
        # make the table
        self.cur.execute('CREATE TABLE bookitems(author TEXT , title TEXT, file_size INTEGER)')
    
        # Add some hardcoded titles to the sqllite database (remove if you dont want to do this)
        inserts = [
            ("jon johnson","his name was seth", 100),
            ("shane macian","blue clouds grey skys", 200),
            ("xhin lee","kung fu kickings", 10),
            ("jan shindler","monitoral obsession", 70),
            ("anis kumar","clash of gods", 90),
            ("anil reddy","color red", 100),
            ("iain janssen","my dog jonny", 2220),
            ("johhny gault","death to columbus", 110),
            ("ryddian evans","high peak", 50),
            ("juan wayne jones","the orange genie", 90)
        ]
        # execute many data 
        self.cur.executemany('INSERT INTO bookitems values(?, ?, ?)', inserts)

        # print table data
        self.cur.execute('SELECT * FROM bookitems')
        for row in cur:
            print(row)
        self.cur.close()
        self.comitt_todb()
        
    def add_sql_table(self, author, titel, fsize):
 
        self.cur = self.conn.cursor() 
        # Add the record
        inserts = [(author, titel, fsize) ]
        # execute many data 
        self.cur.executemany('INSERT INTO bookitems values(?, ?, ?)', inserts)

        # print table data
        self.cur.execute('SELECT * FROM bookitems')
        for row in self.cur:
            print(row)
        self.cur.close()
        self.comitt_todb()
        
    def comitt_todb(self):
        self.conn.commit()
            
    def close_conn_todb(self):
        self.conn.close()

    def open_conn_todb(self):
        self.conn = sqlite3.connect(self.dbname) 

if __name__ == "__main__":

    rf = csvDB()                                                                  # create a csv record db object and the load the csv file to the object data
    print("len", len(rf))
    pprint(rf.find_by_author("authors_name"))                                     # perfrom searches on the information 
    pprint(rf.find_by_title(keyword="searching_the_keyword"))
    pprint(rf.find_by_author("authors_name2",size_order=False))	
    pprint(rf.find_by_author("authors_name2",limit=1))	
    rf.add_title_to_db("title_to_add")
    rf.add_author_to_db("authors_name")
    rf.close_conn_todb()