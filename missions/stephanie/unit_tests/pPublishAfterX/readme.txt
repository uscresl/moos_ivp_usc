To run:
pAntler pPublishAfterX.moos

This should show 2 windows, one for pLogger (which is generating log files of
the MOOSDB content), and one for pPublishAfterX, which shows process output.

Then do:
uPokeDB pPublishAfterX.moos EXAMPLE=test
and monitor using uMS, or check the log file after.

