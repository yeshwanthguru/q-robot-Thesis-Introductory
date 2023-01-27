import tkinter as tk
import rospy

def on_select(event):
    topic_name = listbox.get(listbox.curselection())
    try:
        #initialize node 
        rospy.init_node('Topic_detector', anonymous=True)
        # check if topic is published or not 
        if topic_name in rospy.get_published_topics():
            label.config(text="Topic detected", bg="green")
            print("Topic detected:", topic_name)
        else:
            label.config(text="Topic not detected", bg="red")
            print("Topic not detected:", topic_name)
    except rospy.ROSInterruptException:
        pass

root = tk.Tk()
root.title("ROS Topic Selector")

listbox = tk.Listbox(root)
listbox.pack()

# Populate the listbox with available topics
topics = rospy.get_published_topics()
for topic in topics:
    listbox.insert(tk.END, topic[0])

# Populate the listbox with available topics
topics = rospy.get_published_topics()
for topic in topics:
    listbox.insert(tk.END, topic[0])

label = tk.Label(root, text="Select a topic from the list")
label.pack()

root.mainloop()