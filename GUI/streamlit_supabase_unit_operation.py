#!/usr/bin/python
#
# Example of interfacting to supabase database for creating operations that the sequencer can perform
# the example is for example a blender mixing 4 chosen items and quantities
#
import streamlit as st                                                            # use the streamlit gui
import psycopg2
import datetime
import pandas as pd
from dateutil.relativedelta import relativedelta

# ===================== Classes relating to each database table =================================

# class list the possible number of operations of sequence that can be chosen for the blender
class Blender:
    def __init__(self,o1=" ",o2=" ",o3=" ",o4=" "):  
        self.prod_id = "batch_id_for_product"    
        self.op1 = o1                                    # first chosen operation name
        self.qty1 = 0                                    # qty to blend you could make a string "[spt]::[pipe charge qty]" e.g. flowmeter valve distance
        self.op2 = o2
        self.qty2 = 0       
        self.op3 = o3
        self.qty3 = 0 
        self.op4 = o4
        self.qty4 = 0 

# define each operation (valve entry and exit state for vessel) and its name
class unitOperation:
    def __init__(self): 
        self.unit_operation = "name for operation"        # example add milk
        self.valve_entry_pos = 10                         # sequence output states [ 0, 0, 0, 1, 0, 1, 0 ]
        self.valve_exit_pos = 4                           # sequence output states [ 0, 0, 0, 0, 1, 0, 0 ]  
        
def main():
    #valves = [st.secrets["VV001 inlet"],st.secrets["VV002 outlet"],st.secrets["VV003 milk"],st.secrets["VV004 banana"],st.secrets["VV005 rasberry"],st.secrets["VV006 vanilla"],st.secrets["VV007 choco"],st.secrets["MM001 mixer"]]

    st.subheader("~~~~Milk Blender~~~~")

    tab_input, tab_action, tab_edit, tab_run = st.tabs(["Product", "Operation", "Delete", "Run"])   

    categories = get_categories()
    blend_sequence = Blender()
    uo.unitOperation()
    
    # define each product to make in the blender    
    with tab_input:
        col_date, col_item, col_cat1, col_cat2, col_cat3, col_cat4 = st.columns(6)
        with col_date:
            date = st.date_input("Date", value=None)
        with col_item:
            blend_sequence.prod_id = st.text_input("Product Id")
        with col_cat1:
            blend_sequence.op1 = st.selectbox("first blend_operation", categories)
        with col_cat2:
            blend_sequence,op2 = st.selectbox("second blend_operation", categories)
        with col_cat3:
            blend_sequence,op3 = st.selectbox("third blend_operation", categories)
        with col_cat4:
            blend_sequence.op4 = st.selectbox("last blend_operation", categories)
           
        col_qty1 ,col_qty2, col_qty3, col_qty4 = st.columns(4)
        with col_qty1:
            blend_sequence.qty1 = st.text_input("Qty1")
        with col_qty2:
            blend_sequence.qty1 = st.text_input("Qty2")
        with col_qty3:
            blend_sequence.qty1 = st.text_input("Qty3")
        with col_qty4:
            blend_sequence.qty1 = st.text_input("Qty4")
    
        if st.button("Create New Recipe"):
            try:
                if not blend_sequence.qty1:
                    st.warning("Please enter the qty for the first operation")
                elif not blend_sequence.prod_id:
                    st.warning("Enter the product Id")
                elif not blend_sequence.op1:
                    st.warning("Select the first unit operation to perform")
                else:
                    try:
                        add_new_product(date, blend_sequence)
                    except Exception as e:
                        st.error(f"Recipe Creation failed\n{e}")
                    else:
                        st.experimental_rerun()
                    st.success("Recipe Creation was successful.")
            except Exception as e:
                st.error(f"Recipe Creation failed\n{e}")

    with tab_action:
        cre_date, op_name, begin_seq, end_seq = st.columns(4)

        # for the operation
        with cre_date:
            date = st.date_input("Date", value=None)
        with col_item:
            uo.unit_operation = st.text_input("Unit Operation Name")
        with begin_seq:
            # uo.valve_entry_pos = st.radio("inlet valve table", valves, horizontal=True) only 1 option can be selected we might need mor than 1 valve
            st.write('Select if open/running in the step entry valve table:')
            option_1 = st.checkbox('VV001 inlet')
            option_2 = st.checkbox('VV002 outlet')
            option_3 = st.checkbox('VV003 milk')
            option_4 = st.checkbox('VV004 banana')
            option_5 = st.checkbox('VV005 rasberry')
            option_6 = st.checkbox('VV006 vanilla')
            option_7 = st.checkbox('VV007 choc')
            option_8 = st.checkbox('MM001 mixer')
            v = 0
            if option_1 == True: v = 1
            if option_2 == True: v |= 2
            if option_3 == True: v |= 4
            if option_4 == True: v |= 8
            if option_5 == True: v |= 16
            if option_6 == True: v |= 32
            if option_7 == True: v |= 64   
            if option_8 == True: v |= 128
            uo.valve_entry_pos = v           
        with end_seq:
            #uo.valve_exit_pos = st.radio("outlet valve table", valves, horizontal=True)
            st.write('Select for open/running in the step exit valve table:')
            option_1 = st.checkbox('VV001 inlet')
            option_2 = st.checkbox('VV002 outlet')
            option_3 = st.checkbox('VV003 milk')
            option_4 = st.checkbox('VV004 banana')
            option_5 = st.checkbox('VV005 rasberry')
            option_6 = st.checkbox('VV006 vanilla')
            option_7 = st.checkbox('VV007 choc')
            option_4 = st.checkbox('MM001 mixer')
            v = 0
            if option_1 == True: v = 1
            if option_2 == True: v |= 2
            if option_3 == True: v |= 4
            if option_4 == True: v |= 8
            if option_5 == True: v |= 16
            if option_6 == True: v |= 32
            if option_7 == True: v |= 64   
            if option_8 == True: v |= 128
            uo.valve_exit_pos = v 
        if st.button("Create New Unit Operation"):
            try:
                if not uo.unit_operation:
                    st.warning("Please enter the name of the operation")
                else:
                    try:
                        add_new_operation(date, uo)
                    except Exception as e:
                        st.error(f"Unit Operation Creation failed\n{e}")
                    else:
                        st.experimental_rerun()
                    st.success(f"Unit Operation {uo.unit_operation} Creation was successful.")
            except Exception as e:
                st.error(f"Unit Operation Creation failed\n{e}")

    with tab_edit:
        product_df = get_product_history()
        uop_df = get_unit_op_history()
        product_df = product_df.sort_values("Date")
        col_delete, col_display, col_display1 = st.columns(3)

        with col_delete:
            try:
                id = st.number_input("Enter the product id to delete", product_df["id"].min(),product_df["id"].max(),value = product_df["id"].max())
                st.subheader(f"{product_df[product_df['id']==id].iat[0,2]}")
                if st.button("Delete"):
                    delete_product(id)
                    st.experimental_rerun()
            except:
                pass

        with col_display:
            product_df = product_df.set_index("id")
            st.dataframe(product_df.iloc[::-1],height=250)
        with col_display1:
            st.dataframe(uop_df.iloc[::-1],height=250)

    with tab_run:
        product_df = get_product_history()
        uop_df = get_unit_op_history()
        product_df = product_df.sort_values("Date")
        col_run, col_display, col_display1 = st.columns(3)

        with col_run:
            try:
                id = st.number_input("Enter the product id to delete", product_df["id"].min(),product_df["id"].max(),value = product_df["id"].max())
                st.subheader(f"{product_df[product_df['id']==id].iat[0,2]}")
                if st.button("Run"):
                    run_data = get_product(id)
                    # data to download or send to controller or use within this program
                    print(f"making {run_data['id']} op={run_data['Uop1']} qty={run_data['qty1']}")
                    print(f"                        op={run_data['Uop2']} qty={run_data['qty2']}")
                    print(f"                        op={run_data['Uop3']} qty={run_data['qty3']}")
                    print(f"                        op={run_data['Uop4']} qty={run_data['qty4']}")
                    st.dataframe(run_data.iloc[::-1],height=250)
            except:
                pass

        with col_display:
            product_df = product_df.set_index("id")
            st.dataframe(product_df.iloc[::-1],height=250)
        with col_display1:
            st.dataframe(uop_df.iloc[::-1],height=250)

# connect to database
def conn_supabase():
    ip = st.secrets["host"]
    port = st.secrets["port"]
    dbname = st.secrets["dbname"]
    user = st.secrets["user"]
    pw = st.secrets["password"]

    return f"host={ip} port={port} dbname={dbname} user={user} password={pw}"

# list each of the possible unit operations
def get_categories():
    sql = f"""
        SELECT op_name
        FROM products_info.operations
        """

    with psycopg2.connect(conn_supabase()) as conn:
        with conn.cursor() as cur:
            cur.execute(sql)
            data = cur.fetchall()

    categories = [update_blend_sequence[0] for update_blend_sequence in data]

    return categories

# add a new product to manufacture         
def add_new_product(date, blend_sequence):
    sql = f"""
        INSERT INTO products_info.batch_seq
            (date, blend_sequence_id, op1, qty1,  op2, qty2,  op3, qty3,  op4, qty4 )
        VALUES (
            \'{date}\',
            \'{blend_sequence.prod_id}\',
            \'{blend_sequence.op1}\',
            {blend_sequence.qty1},
            \'{blend_sequence.op2}\',
            {blend_sequence.qty2},            
            \'{blend_sequence.op3}\',
            {blend_sequence.qty3},           
            \'{blend_sequence.op4}\',
            {blend_sequence.qty4}
        )
        """
    with psycopg2.connect(conn_supabase()) as conn:
        with conn.cursor() as cur:
            cur.execute(sql)
        conn.commit()

# add a new unit operation (additive)
def add_new_operation(date, op):
    sql = f"""
        INSERT INTO products_info.operations
            (date, op_name, entry, exit )
        VALUES (
            \'{date}\',
            \'{op.unit_operation}\',
            \'{op.valve_entry_pos}\',
            \'{op.valve_exit_pos}\'
        )
        """
    with psycopg2.connect(conn_supabase()) as conn:
        with conn.cursor() as cur:
            cur.execute(sql)
        conn.commit()

# get the product data
def get_product_history():
    sql = f"""
        SELECT
            blend_sequence_id,
            date,
            op1,
            qty1,
            op1,
            qty2,
            op3,
            qty3,
            op4,
            qty4
        FROM products_info.batch_seq
    """
    with psycopg2.connect(conn_supabase()) as conn:
        with conn.cursor() as cur:
            cur.execute(sql)
            data = cur.fetchall()

    colnames =["id", "Date", "Unit op1", "qty1", "Unit op2", "qty2", "Unit op3", "qty3", "Unit op4", "qty4" ]
    product_df = pd.DataFrame(data,columns=colnames)

    return product_df

# get the unit operation data
def get_unit_op_history():
    sql = f"""
        SELECT
            op_name,
            date,
            entry,
            exit
        FROM products_info.operations
    """
    with psycopg2.connect(conn_supabase()) as conn:
        with conn.cursor() as cur:
            cur.execute(sql)
            data = cur.fetchall()

    colnames =["id", "Date", "step entry output state", "step exit output state" ]
    op_df = pd.DataFrame(data, columns=colnames)

    return op_df

# deletes the product specified
def delete_product(id):
    sql = f"""
        DELETE FROM products_info.batch_seq
        WHERE blend_sequence_id = {id}
        """
    with psycopg2.connect(conn_supabase()) as conn:
        with conn.cursor() as cur:
            cur.execute(sql)
        conn.commit()

# gets the product specified for running on the sequencer
def get_product(id):
    sql = f"""
        SELECT * FROM products_info.batch_seq
        WHERE blend_sequence_id = {id}
        """
    with psycopg2.connect(conn_supabase()) as conn:
        with conn.cursor() as cur:
            cur.execute(sql)
            data = cur.fetchall()
    colnames =["id", "Date", "Uop1", "qty1", "Uop2", "qty2", "Uop3", "qty3", "Uop4", "qty4" ]
    product_df = pd.DataFrame(data, columns=colnames)

    return product_df

if __name__ == '__main__':
    main()