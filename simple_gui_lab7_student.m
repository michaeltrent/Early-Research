function simple_gui_lab7_student
% SIMPLE_GUI_LAB7 Select a data set from the pop-up menu, then
% click one of the plot-type push buttons. Clicking the button
% plots the selected data in the axes.
 
   %  Create and then hide the GUI as it is being constructed.
   f = figure('Visible','off','Position',[360,500,600,300]);
 
   %  Construct the components.
   hposition = uicontrol('Style','pushbutton','String','Position',...
          'Position',[315,220,70,25],...
          'Callback',{@positionbutton_Callback});
   hvelocity = uicontrol('Style','pushbutton','String','Velocity',...
          'Position',[315,180,70,25],...
          'Callback',{@velocitybutton_Callback},'Visible','on');
   hacceleration = uicontrol('Style','pushbutton',...
          'String','Acceleration',...
          'Position',[315,135,70,25],...
          'Callback',{@accelerationbutton_Callback},'visible','on'); 
   htext = uicontrol('Style','text','String','Select Data',...
          'Position',[325,80,200,20],'Visible','off');
   htext_t = uicontrol('Style','text','String','Select Data',...
          'Position',[325,40,200,20],'Visible','off');
  ha = axes('Units','Pixels','Position',[80,60,200,185]); 
  align([hposition,hvelocity,hacceleration,htext,htext_t],'Center','None');
  
   % Create the data to plot.
   [t,r,v,a]=minjerk([0 0],[0 20],1,0.001);
   
   %Get max position and save it
   maxr=max(r(:,2));
   %Get time of max position and save it
   t_maxr=t(r(:,2)==max(r(:,2)));
   
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%Do the same for velocity and acceleration here:
   
   
   % Get max velocity
   maxv = max(v(:,2));
  %Get time of max velocity
  t_maxv = t(v(:,2) == max(v(:,2)));
  
  % And again for acceleration
  
  maxa = max(a(:,2));
  % time at max a
  t_maxa = t(a(:,2) == maxa);
   

   
      
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % Initialize the GUI.
   % Change units to normalized so components resize 
   % automatically.
   
   %set([f,ha,hsurf,hmesh,hcontour,htext,hpopup],...
   set([f,ha,hposition,hvelocity,hacceleration,htext,htext_t],...
   'Units','normalized');
   %Create a plot in the axes.
   axes(ha)
   plot(t,r(:,2));
   title('Position')
   xlabel('Time (s)')
   ylabel('Position (m)')
      
   % Assign the GUI a name to appear in the window title.
   set(f,'Name','Simple GUI')
   % Move the GUI to the center of the screen.
   movegui(f,'center')
   % Make the GUI visible.
   set(f,'Visible','on');
 
   %  Callbacks for simple_gui. These callbacks automatically
   %  have access to component handles and initialized data 
   %  because they are nested at a lower level.
 
  
   % Push button callbacks. Each callback plots current_data in
   % the specified plot type.
 
   function positionbutton_Callback(source,eventdata) 
   % Display position plot of the currently selected data.
      plot(t,r(:,2))
      title('Position')
      xlabel('Time (s)')
      ylabel('Position (m)')
      set(htext,'String',sprintf('max position is %4.2f m',maxr),'Visible','On')
      set(htext_t,'String',sprintf('Corresponding Time is %4.2f s',t_maxr),'Visible','On')
   end

   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   %%Do the same for velocity and acceleration here:
      function velocitybutton_Callback(source,eventdata) 
   % Display position plot of the currently selected data.
      plot(t,v(:,2))
      title('Velocity')
      xlabel('Time (s)')
      ylabel('Velocity (m/s)')
      set(htext,'String',sprintf('max velocity is %4.2f m/s',maxv),'Visible','On')
      set(htext_t,'String',sprintf('Corresponding Time is %4.2f s',t_maxv),'Visible','On')
      end
   

     function accelerationbutton_Callback(source,eventdata) 
   % Display position plot of the currently selected data.
      plot(t,a(:,2))
      title('Acceleration')
      xlabel('Time (s)')
      ylabel('Acceleration (m/s^2)')
      set(htext,'String',sprintf('max acceleration is %4.2f m/s^2',maxa),'Visible','On')
      set(htext_t,'String',sprintf('Corresponding Time is %4.2f s',t_maxa),'Visible','On')
     end

      
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 

 
end 