classdef Gripper < RobotBaseClass
  

    properties(Access = public)   
        plyFileNameStem = 'Gripper';
    end
    
    methods
%% Constructor
        function self = Gripper(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();

            drawnow
        end
%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0,'a',0.02,'alpha',0,'qlim',deg2rad([-360 360]), 'offset', 0); % PRISMATIC Link          
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
