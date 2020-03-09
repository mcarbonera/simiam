A MATLAB-based Educational Bridge Between Theory and Practice in Robotics.

ps (mcarbonera):

This fork has the intent to add a controller that implements the navigation task with a Mamdani Fuzzy Inference System.

This work will be the practical part of my graduation thesis in Mobile Robotics. In this work I'll explore behavior arbitration (with hybrid system) and behavior fusion (with Fuzzy Inference System).

I'm using a different robot that I named Jubileu. The class "Jubileu.m" is inside the folder "robot". The supervisor is inside folder "controller/jubileu", with the name "JSupervisor.m".
	
The folders were organized in a way that documents the steps involved on design process. 

The steps are: 
	
	-> Follow Angle (Folder "SeguirAngulo")
	-> Go To Goal (Folder "IrParaMeta")
	-> Obstacle Avoidance (Folder "EvitarObstaculo")
	-> GTG and OA combined (Folder "Mesclar_EO_IPM")
	-> Follow Wall (Folder "FollowWall")
	-> Complete navigation (Folder "Navegacao")