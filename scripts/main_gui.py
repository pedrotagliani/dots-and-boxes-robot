import customtkinter

customtkinter.set_appearance_mode('system')  # Modes: system (default), light, dark
customtkinter.set_default_color_theme('blue')  # Themes: blue (default), dark-blue, green

class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()

        self.title('Dots and Boxes Robot')
        self.iconbitmap('assets/icon/icon1.ico')
        self.geometry('960x540')



app = App()
app.mainloop()

