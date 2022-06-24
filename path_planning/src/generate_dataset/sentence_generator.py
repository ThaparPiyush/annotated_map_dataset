import random

Imperatives = [' go to ', ' reach ', ' enter ', ' pass through ']

Connectives = [' followed by ', ' then ', ' and then ', ' afterwards ', ' and next ', ' after ', ' via ']

Places = ['Room_', 'Door_']

Objects = ['Table_']

Sentences = {}

for Imperative in Imperatives:
    for Place in Places:
        num = str(random.randint(1,4))
        Sentence = Imperative + Place + num
        Waypoints = ((Place + num),)
        Sentences[Sentence] = Waypoints

for Imperative in Imperatives:
    for Object in Objects:
        num = str(random.randint(1,4))
        Sentence = Imperative + Object + num
        Waypoints = ((Object + num),)
        Sentences[Sentence] = Waypoints

for Imperative in Imperatives:
    for Place in Places:
        for Connective in Connectives:
            for Object in Objects:
                num = str(random.randint(1,4))
                if Connective == ' followed by ':
                    Sentence = Imperative + Place + num + Connective + Object + num
                    Waypoints = ((Object + num), (Place + num))
                elif Connective == ' after ' or Connective == ' via ':
                    Sentence = Imperative + Place + num + Connective + Object + num
                    Waypoints = ((Object + num), (Place + num))
                else:
                    Sentence = Imperative + Place + num + Connective + Imperative + Object + num
                    Waypoints = ((Place + num), (Object + num))
                Sentences[Sentence] = Waypoints

#for Sentece in Sentences:
#    print(Sentence, ' -> ', Sentences[Sentence])
#print(len(Sentences))
#

Sentences1 = {}
for index1, (key1, value1) in enumerate(Sentences.items()):
    for index2, (key2, value2) in enumerate(Sentences.items()):
        if(index1 != index2):
            Waypoints = value1 + value2
            Sentence = key1 + ". Then " + key2
            Sentences1[Sentence] = Waypoints

for index, (key, value) in enumerate(Sentences1.items()):
    print(key, ' -> ', value)
print(len(Sentences1))

#Sentences2 = {}
#for index1, (key1, value1) in enumerate(Sentences1.items()):
#    for index2, (key2, value2) in enumerate(Sentences1.items()):
#        if(index1 != index2):
#            Waypoints = value1 + value2
#            Sentence = key1 + ". Then " + key2
#            Sentences2[Sentence] = Waypoints
#            print(Sentence, ' -> ', Sentences2[Sentence])
#            print(len(Sentences2))
#
#print(len(Sentences2))


#SentenceConnectives = ['. Next ', '. Furthermore ', '. Moreover ', '. Additionally ', '. In addition to that ', '. As well ']
