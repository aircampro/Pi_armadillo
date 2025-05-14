#
# Example of increasing/decreasing stock using main form buttons with the progress indicated as a bar graph on a sub window
#
#
import tkinter as tk
from tkinter import ttk
from dataclasses import dataclass
from typing import Callable, Dict, List
import threading

@dataclass
class Subscriber:
    callback: Callable
    sync: bool = True

class EventManager:
    def __init__(self):
        self._subscribers: Dict[str, List[Subscriber]] = {}
        self._lock = threading.Lock()
    
    def subscribe(self, event_name: str, callback: Callable, sync: bool = True) -> None:
        """ subscriber """
        with self._lock:
            if event_name not in self._subscribers:
                self._subscribers[event_name] = []
            self._subscribers[event_name].append(Subscriber(callback, sync))
    
    def unsubscribe(self, event_name: str, callback: Callable) -> None:
        """ un-subscriber """
        with self._lock:
            if event_name in self._subscribers:
                self._subscribers[event_name] = [
                    sub for sub in self._subscribers[event_name] 
                    if sub.callback != callback
                ]
    
    def publish(self, event_name: str, *args, **kwargs) -> None:
        """ publisher """
        with self._lock:
            if event_name not in self._subscribers:
                return
            subscribers = self._subscribers[event_name].copy()
        
        for subscriber in subscribers:
            if subscriber.sync:
                subscriber.callback(*args, **kwargs)
            else:
                threading.Thread(
                    target=subscriber.callback,
                    args=args,
                    kwargs=kwargs
                ).start()

# main parent window
class MainWindow:
    def __init__(self, event_manager: EventManager):
        self.root = tk.Tk()
        self.root.title(" Stock Control System ")
        self.event_manager = event_manager
        
        # set the default stock at 100
        self.stock = 100
        
        # create frame
        frame = ttk.Frame(self.root, padding="10")
        frame.grid()
        
        ttk.Label(frame, text="Stock Qty:").grid(row=0, column=0)
        self.stock_label = ttk.Label(frame, text=str(self.stock))
        self.stock_label.grid(row=0, column=1)
        
        ttk.Button(frame, text="Make Stock(+10)", command=self.stock_in).grid(row=1, column=0)
        ttk.Button(frame, text="Use Stock(-10)", command=self.stock_out).grid(row=1, column=1)
        
        # progress button
        ttk.Button(
            frame, 
            text="Show Stock as Bar Graph", 
            command=self.open_sub_window
        ).grid(row=2, column=0, columnspan=2)
    
    def stock_in(self):
        self.stock += 10
        self.stock_label.config(text=str(self.stock))
        # publish event with new stock update
        self.event_manager.publish("stock_updated", self.stock)
    
    def stock_out(self):
        if self.stock >= 10:
            self.stock -= 10
            self.stock_label.config(text=str(self.stock))
            # publish event with new stock update
            self.event_manager.publish("stock_updated", self.stock)
    
    def open_sub_window(self):
        SubWindow(self.event_manager, self.stock)

# child window
class SubWindow:
    def __init__(self, event_manager: EventManager, initial_stock: int):
        self.window = tk.Toplevel()
        self.window.title("stock levels")
        self.event_manager = event_manager
        
        frame = ttk.Frame(self.window, padding="10")
        frame.grid()
        
        ttk.Label(frame, text="Ammount of Stock").grid(row=0, column=0, columnspan=2)
        
        self.gauge = ttk.Progressbar(
            frame, 
            length=200, 
            mode='determinate',
            maximum=200
        )
        self.gauge.grid(row=1, column=0, columnspan=2)
        
        self.stock_label = ttk.Label(frame, text=f"{initial_stock} items ")
        self.stock_label.grid(row=2, column=0, columnspan=2)
        
        # sunscribe to the event
        self.event_manager.subscribe("stock_updated", self.update_stock)
        
        # set to delete window on close
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)
        
        # update stock
        self.update_stock(initial_stock)
    
    def update_stock(self, stock: int):
        self.stock_label.config(text=f"{stock} items ")
        self.gauge['value'] = stock
    
    def on_closing(self):
        # close child sub window
        self.event_manager.unsubscribe("stock_updated", self.update_stock)
        self.window.destroy()

def main():
    event_manager = EventManager()
    main_window = MainWindow(event_manager)
    main_window.root.mainloop()

if __name__ == "__main__":
    main()
