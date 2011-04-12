with PolyORB_HI.Errors;
with PolyORB_HI_Generated.Deployment;
with PolyORB_HI.Streams;
with PolyORB_HI.Utils;
with System;

package PolyORB_HI_Drivers_GRSPW is

   use PolyORB_HI.Errors;
   use PolyORB_HI_Generated.Deployment;
   use PolyORB_HI.Streams;

   procedure Initialize (Name_Table : PolyORB_HI.Utils.Naming_Table_Type);

   procedure Receive;

   function Send
     (Node    : Node_Type;
      Message : Stream_Element_Array;
      Size    : Stream_Element_Offset)
     return Error_Kind;

   task Idle_Task is
      --  Dummy idle task to work around an issue in the GRSPW driver
      --  in gnatforleon: if no task executes, the node goes in sleep
      --  mode, and cannot be awaken when a packet comes in. This task
      --  simulates a constant workload to prevent the node to
      --  hibernate.

     pragma Priority (System.Priority'First);
   end Idle_Task;

end PolyORB_HI_Drivers_GRSPW;
