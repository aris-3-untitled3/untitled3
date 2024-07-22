def play_mp3(self):
    pygame.mixer.init()
    pygame.mixer.music.load("/home/jchj/Untitled3/src/untitled3/UI/cat_like1b.mp3")
    pygame.mixer.music.play()