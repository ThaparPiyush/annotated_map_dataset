import random

Imperatives = [' go to ', ' reach ']#, ' enter ', ' pass through ']

# Connectives = [' followed by ', ' then ', ' and then ', ' afterwards ', ' and next ', ' after ', ' via ']
Connectives = [' then ', ' after that ']

Sentences = {}

## Check no of places in map
#   nRooms = nTables = nDoors = 0
#   for loc in locations_parsed:
#       if (sen[0][0:-2] == 'Room'):
#           nRooms += 1
#       elif (sen[0][0:-2] == 'Table'):
#           nTables += 1
#       elif (sen[0][0:-2] == 'Door'):
#           nDoors += 1
class sentences:
    def __init__(self, locations):
        self.Sentences = {}
        self.places = []
        self.nDoors = self.nRooms = self.nTables = 0
        for loc in locations:
            if (loc[0][0:-2] == 'Room'):
                self.nRooms +=1
            elif (loc[0][0:-2] == 'Table'):
                self.nTables += 1
            elif (loc[0][0:-2] == 'Door'):
                self.nDoors += 1


        for door in range (0, self.nDoors):
            self.places.append('Door_' + str(door+1))
        for room in range(0, self.nRooms):
            self.places.append('Room_' + str(room+1))
        for table in range(0, self.nTables):
            self.places.append('Table_' + str(table+1))

        for imperative in Imperatives:
            for place in self.places:
                sentence = imperative + place
                waypoints = ((place),)
                self.Sentences[sentence] = waypoints
    def returnSentences(self):
        return self.Sentences



#nDoors = nTables = nRooms = 0
#
#for Imperative in Imperatives:
#    for Place in Places:
#        num = str(random.randint(1,4))
#        Sentence = Imperative + Place + num
#        Waypoints = ((Place + num),)
#        Sentences[Sentence] = Waypoints
#
#for Imperative in Imperatives:
#    for Object in Objects:
#        num = str(random.randint(1,4))
#        Sentence = Imperative + Object + num
#        Waypoints = ((Object + num),)
#        Sentences[Sentence] = Waypoints
#
#for Imperative in Imperatives:
#    for Place in Places:
#        for Connective in Connectives:
#            for Object in Objects:
#                num = str(random.randint(1,4))
#                if Connective == ' followed by ':
#                    Sentence = Imperative + Place + num + Connective + Object + num
#                    Waypoints = ((Object + num), (Place + num))
#                elif Connective == ' after ' or Connective == ' via ':
#                    Sentence = Imperative + Place + num + Connective + Object + num
#                    Waypoints = ((Object + num), (Place + num))
#                else:
#                    Sentence = Imperative + Place + num + Connective + Imperative + Object + num
#                    Waypoints = ((Place + num), (Object + num))
#                Sentences[Sentence] = Waypoints
#
##for Sentece in Sentences:
##    print(Sentence, ' -> ', Sentences[Sentence])
##print(len(Sentences))
##
#
#Sentences1 = {}
#for index1, (key1, value1) in enumerate(Sentences.items()):
#    for index2, (key2, value2) in enumerate(Sentences.items()):
#        if(index1 != index2):
#            Waypoints = value1 + value2
#            Sentence = key1 + ". Then " + key2
#            Sentences1[Sentence] = Waypoints
#
#for index, (key, value) in enumerate(Sentences.items()):
#    print(key, ' -> ', value)
#    print((value[0] == "Table_3"))
#print(len(Sentences))
#
#
##Sentences2 = {}
##for index1, (key1, value1) in enumerate(Sentences1.items()):
##    for index2, (key2, value2) in enumerate(Sentences1.items()):
##        if(index1 != index2):
##            Waypoints = value1 + value2
##            Sentence = key1 + ". Then " + key2
##            Sentences2[Sentence] = Waypoints
##            print(Sentence, ' -> ', Sentences2[Sentence])
##            print(len(Sentences2))
##
##print(len(Sentences2))
#
#
##SentenceConnectives = ['. Next ', '. Furthermore ', '. Moreover ', '. Additionally ', '. In addition to that ', '. As well ']
