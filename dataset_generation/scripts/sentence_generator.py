import random

Imperatives = [' go to ', ' reach ']#, ' enter ', ' pass through ']

# Connectives = [' followed by ', ' then ', ' and then ', ' afterwards ', ' and next ', ' after ', ' via ']
Connectives = [' then ', ' after that ']

Sentences = {}

## Check no of Places in map
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
        self.Places = []
        self.nDoors = self.nRooms = self.nTables = 0
        self.object_list = ['Table', 'Sofa', 'Almirah', 'Lamp', 'Carpet', 'Fan', 'Computer', 'Tea', 'Plant', 'Telephone']
        for loc in locations:
            if '_' not in loc[0]:
                self.Places.append(loc[0])
        #for loc in locations:
        #    if (loc[0][0:-2] == 'Room'):
        #        self.nRooms +=1
        #    elif (loc[0][0:-2] == 'Table'):
        #        self.nTables += 1
            # elif (loc[0][0:-2] == 'Door'):
            #     self.nDoors += 1


        # for door in range (0, self.nDoors):
        #     self.Places.append('Door_' + str(door+1))
        #for room in range(0, self.nRooms):
        #    self.Places.append('Room_' + str(room+1))
        #for table in range(0, self.nTables):
        #    self.Places.append(self.object_list[table+1])

        for imperative in Imperatives:
            for connective in Connectives:
                for place1 in range(0, len(self.Places)-1):
                    remaining_places = self.Places[:place1] + self.Places[(place1+1):]
                    for place2 in range(0, len(remaining_places)-1):
                        sentence = imperative + self.Places[place1] + connective + imperative + remaining_places[place2]
                        waypoints = (self.Places[place1], remaining_places[place2])
                        self.Sentences[sentence] = waypoints
        #for imperative in Imperatives:
        #   for place in self.Places:
        #       sentence = imperative + place
        #       waypoints = ((place),)
        #       self.Sentences[sentence] = waypoints
    def returnSentences(self):
        return self.Sentences

##SentenceConnectives = ['. Next ', '. Furthermore ', '. Moreover ', '. Additionally ', '. In addition to that ', '. As well ']
