#
# Example of tkinter window with categories 
#
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from datetime import datetime
import csv
 
class BookkeepingSandbox:
    def __init__(self, root):
        self.root = root
        root.title("Bookkeeping Sandbox (No Saving Except via CSV)")
        root.geometry("950x650")
 
        self.data = []  # in-memory only
 
        self.categories = [
            "Income",
            "Sales",
            "Donations",
            "COGS",
            "Supplies",
            "Utilities",
            "Rent",
            "Payroll",
            "Misc Expense",
        ]
 
        # ======================
        # ADD FORM
        # ======================
        form_frame = tk.LabelFrame(root, text="Add Transaction", padx=10, pady=10)
        form_frame.pack(fill="x", padx=10, pady=5)
 
        tk.Label(form_frame, text="Date (YYYY-MM-DD):").grid(row=0, column=0, sticky="w")
        tk.Label(form_frame, text="Description:").grid(row=1, column=0, sticky="w")
        tk.Label(form_frame, text="Amount:").grid(row=2, column=0, sticky="w")
        tk.Label(form_frame, text="Category:").grid(row=3, column=0, sticky="w")
 
        self.entry_date = tk.Entry(form_frame)
        self.entry_desc = tk.Entry(form_frame)
        self.entry_amt = tk.Entry(form_frame)
 
        self.entry_cat = ttk.Combobox(form_frame, values=self.categories, state="readonly")
        self.entry_cat.set("Select Category")
 
        self.entry_date.grid(row=0, column=1)
        self.entry_desc.grid(row=1, column=1)
        self.entry_amt.grid(row=2, column=1)
        self.entry_cat.grid(row=3, column=1)
 
        tk.Button(form_frame, text="Add Transaction", command=self.add).grid(
            row=4, column=0, columnspan=2, pady=5
        )
 
        # ======================
        # FILTER PANEL
        # ======================
        filter_frame = tk.LabelFrame(root, text="Filters", padx=10, pady=10)
        filter_frame.pack(fill="x", padx=10, pady=5)
 
        tk.Label(filter_frame, text="Date From:").grid(row=0, column=0)
        tk.Label(filter_frame, text="Date To:").grid(row=0, column=2)
        tk.Label(filter_frame, text="Category:").grid(row=1, column=0)
        tk.Label(filter_frame, text="Keyword:").grid(row=1, column=2)
 
        self.filter_from = tk.Entry(filter_frame)
        self.filter_to = tk.Entry(filter_frame)
 
        self.filter_cat = ttk.Combobox(filter_frame, values=["All"] + self.categories)
        self.filter_cat.set("All")
 
        self.filter_kw = tk.Entry(filter_frame)
 
        self.filter_from.grid(row=0, column=1)
        self.filter_to.grid(row=0, column=3)
        self.filter_cat.grid(row=1, column=1)
        self.filter_kw.grid(row=1, column=3)
 
        tk.Button(filter_frame, text="Apply Filters", command=self.apply_filters).grid(
            row=0, column=4, rowspan=2, padx=10
        )
        tk.Button(filter_frame, text="Clear Filters", command=self.clear_filters).grid(
            row=0, column=5, rowspan=2, padx=10
        )
 
        # ======================
        # TABLE
        # ======================
        table_frame = tk.Frame(root)
        table_frame.pack(fill="both", expand=True)
 
        columns = ("Date", "Description", "Amount", "Category")
        self.table = ttk.Treeview(table_frame, columns=columns, show="headings")
 
        for col in columns:
            self.table.heading(col, text=col)
            self.table.column(col, width=190)
 
        self.table.pack(fill="both", expand=True)
 
        # ======================
        # RUNNING TOTALS BAR
        # ======================
        totals_frame = tk.Frame(root)
        totals_frame.pack(fill="x", pady=5)
 
        self.total_income_label = tk.Label(totals_frame, text="Income: $0.00", font=("Arial", 11, "bold"))
        self.total_expense_label = tk.Label(totals_frame, text="Expenses: $0.00", font=("Arial", 11, "bold"))
        self.total_net_label = tk.Label(totals_frame, text="Net: $0.00", font=("Arial", 11, "bold"))
 
        self.total_income_label.pack(side="left", padx=15)
        self.total_expense_label.pack(side="left", padx=15)
        self.total_net_label.pack(side="left", padx=15)
 
        # ======================
        # BUTTONS
        # ======================
        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=10)
 
        tk.Button(btn_frame, text="P&L Summary", command=self.show_pl).grid(row=0, column=0, padx=10)
        tk.Button(btn_frame, text="Export CSV", command=self.export_csv).grid(row=0, column=1, padx=10)
        tk.Button(btn_frame, text="Clear All", command=self.clear).grid(row=0, column=2, padx=10)
 
    # ======================================================
    # ADD TRANSACTION
    # ======================================================
    def add(self):
        date = self.entry_date.get().strip()
        desc = self.entry_desc.get().strip()
        amt = self.entry_amt.get().strip()
        cat = self.entry_cat.get().strip()
 
        if not (date and desc and amt and cat and cat != "Select Category"):
            messagebox.showerror("Error", "Please fill all fields.")
            return
 
        try:
            datetime.strptime(date, "%Y-%m-%d")
        except ValueError:
            messagebox.showerror("Error", "Invalid date format.")
            return
 
        try:
            amt = float(amt)
        except ValueError:
            messagebox.showerror("Error", "Amount must be numeric.")
            return
 
        record = (date, desc, amt, cat)
        self.data.append(record)
        self.table.insert("", "end", values=record)
 
        self.update_totals()
 
        self.entry_date.delete(0, tk.END)
        self.entry_desc.delete(0, tk.END)
        self.entry_amt.delete(0, tk.END)
        self.entry_cat.set("Select Category")
 
    # ======================================================
    # UPDATE RUNNING TOTALS
    # ======================================================
    def update_totals(self):
        income = sum(amt for (_, _, amt, _) in self.data if amt >= 0)
        expenses = sum(amt for (_, _, amt, _) in self.data if amt < 0)
        net = income + expenses
 
        self.total_income_label.config(text=f"Income: ${income:,.2f}")
        self.total_expense_label.config(text=f"Expenses: ${expenses:,.2f}")
        self.total_net_label.config(text=f"Net: ${net:,.2f}")
 
    # ======================================================
    # CLEAR ALL
    # ======================================================
    def clear(self):
        self.data = []
        for row in self.table.get_children():
            self.table.delete(row)
        self.update_totals()
 
    # ======================================================
    # FILTERING
    # ======================================================
    def apply_filters(self):
        date_from = self.filter_from.get().strip()
        date_to = self.filter_to.get().strip()
        cat = self.filter_cat.get().strip()
        kw = self.filter_kw.get().strip().lower()
 
        for row in self.table.get_children():
            self.table.delete(row)
 
        for record in self.data:
            d, desc, amt, category = record
 
            if date_from and d < date_from:
                continue
            if date_to and d > date_to:
                continue
            if cat != "All" and category != cat:
                continue
            if kw and kw not in desc.lower():
                continue
 
            self.table.insert("", "end", values=record)
 
        self.update_totals()
 
    def clear_filters(self):
        self.filter_from.delete(0, tk.END)
        self.filter_to.delete(0, tk.END)
        self.filter_kw.delete(0, tk.END)
        self.filter_cat.set("All")
        self.apply_filters()
 
    # ======================================================
    # PROFIT & LOSS POPUP
    # ======================================================
    def show_pl(self):
        income = sum(amt for (_, _, amt, _) in self.data if amt >= 0)
        expenses = sum(amt for (_, _, amt, _) in self.data if amt < 0)
        net = income + expenses
 
        categories = {}
        for _, _, amt, cat in self.data:
            categories.setdefault(cat, 0)
            categories[cat] += amt
 
        report = f"""

=== Profit & Loss ===
 
Total Income: ${income:,.2f}
Total Expenses: ${expenses:,.2f}
Net: ${net:,.2f}
 
=== Category Breakdown ===
"""
        for cat, total in categories.items():
            report += f"{cat}: ${total:,.2f}\n"
 
        messagebox.showinfo("Profit & Loss Summary", report)
 
    # ======================================================
    # EXPORT TO CSV
    # ======================================================
    def export_csv(self):
        if not self.data:
            messagebox.showwarning("No Data", "No transactions to export.")
            return
 
        file_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV Files", "*.csv")],
            title="Save CSV"
        )
 
        if not file_path:
            return
 
        with open(file_path, "w", newline="", encoding="utf-8") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["Date", "Description", "Amount", "Category"])
            writer.writerows(self.data)
 
        messagebox.showinfo("Saved", f"CSV exported successfully:\n{file_path}")
 
 
if __name__ == "__main__":
    root = tk.Tk()
    app = BookkeepingSandbox(root)
    root.mainloop()