How to add a map:

1) generate a geotiff using the python script ecomapper_map_generator.py

2) if needed, move the file to this directory

3) put the min/max lat/lon values you used in a text file, 
   which should have the same name as your image, with extension '.info'
   using lat_north/south/east/west, see any .info file in this dir
*) make sure you put a datum_lat/datum_lon in the .info file. 
   These specify the 'origin' (0,0 in x,y) for your simulation.
   If you have no preference, you can set it to the bottom left coords.

4) if you include it in a simulation, make sure you set the MOOS lat/long origin
   to reasonable values (preferably the same as the datum_lat/lon)

