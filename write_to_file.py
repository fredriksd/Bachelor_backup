#-*- coding:utf-8 -*-
import os

def write_to_file(data, time, name = 'Log_data', first_check = False):
    '''
    Skriver loggdata til egen *.txt - fil. 
    Dataen lagres i format typ csv-fil (comma separated values).
    Ved å sette "first_check" til True vil filens innhold slettes før den blir skrivet til.
    '''
    try:
        if first_check and os.path.isfile("./%s.txt"%name):
            open("%s.txt"%name, "w").close()
            with open('%s.txt' % name, 'a+') as file:
                file.write(str(data) + ',')
                file.write(str(time) + '\n')
                file.close()
        else:
            with open('%s.txt' % name, 'a+') as file:
                file.write(str(data) + ',')
                file.write(str(time) + '\n')
                file.close()
            
    except IOError:
        print "Couldn't write to file..."
            

