def write_to_file(data, time, name = 'Log_data'):
    try:
        with open('%s.txt' % name, 'a+') as file:
            file.write(str(data) + ',')
            file.write(str(time) + '\n')
            file.close
            
    except IOError:
        print "Couldn't write to file..."
            

