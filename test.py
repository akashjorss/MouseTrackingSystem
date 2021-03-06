from Tkinter import *

def show_entry_fields():
   print("First Name: %s\nLast Name: %s" % (e1.get(), e2.get()))

master = Tk()

def task():
    print("Hello")
    master.after(1000,task)
    

Label(master, text="Welcome to the Mouse Tracking System!").grid(row=0)
Label(master, text="Enter the coefficients for:").grid(row=1)
Label(master, text="Point1: ").grid(row=2)
Label(master, text="Point2: ").grid(row=3)
Label(master, text="Eq'n would be: 'c1*P1 + c2*P2' ").grid(row=4)

e1 = Entry(master)
e2 = Entry(master)

e1.grid(row=2, column=1)
e2.grid(row=3, column=1)

Button(master, text='Quit', command=master.quit).grid(row=4, column=2, sticky=W, pady=2)
Button(master, text='Enter', command=show_entry_fields).grid(row=4, column=1, sticky=W, pady=4)

master.after(1000,task)
master.mainloop()

