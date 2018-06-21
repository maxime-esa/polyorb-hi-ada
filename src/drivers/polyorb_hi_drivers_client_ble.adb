------------------------------------------------------------------------------
--                                                                          --
--                          PolyORB HI COMPONENTS                           --
--                                                                          --
--         P O L Y O R B _ H I _ D R I V E R S _ C L I E N T _ B L E        --
--                                                                          --
--                                 B o d y                                  --
--                                                                          --
--                   Copyright (C) 2012-2018 ESA & ISAE.                    --
--                                                                          --
-- PolyORB-HI is free software; you can redistribute it and/or modify under --
-- terms of the  GNU General Public License as published  by the Free Soft- --
-- ware  Foundation;  either version 3,  or (at your option) any later ver- --
-- sion. PolyORB-HI is distributed in the hope that it will be useful, but  --
-- WITHOUT ANY WARRANTY; without even the implied warranty of               --
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                     --
--                                                                          --
-- As a special exception under Section 7 of GPL version 3, you are granted --
-- additional permissions described in the GCC Runtime Library Exception,   --
-- version 3.1, as published by the Free Software Foundation.               --
--                                                                          --
-- You should have received a copy of the GNU General Public License and    --
-- a copy of the GCC Runtime Library Exception along with this program;     --
-- see the files COPYING3 and COPYING.RUNTIME respectively.  If not, see    --
-- <http://www.gnu.org/licenses/>.                                          --
--                                                                          --
--              PolyORB-HI/Ada is maintained by the TASTE project           --
--                      (taste-users@lists.tuxfamily.org)                   --
--                                                                          --
------------------------------------------------------------------------------

with Ada.Interrupts;       use Ada.Interrupts;
with Ada.Interrupts.Names; use Ada.Interrupts.Names;
with Ada.Streams;
with Ada.Exceptions;
with Ada.Real_Time;
with Interfaces;
with Ada.Unchecked_Conversion;
with System;
with System.IO;

with PolyORB_HI.Messages;
with PolyORB_HI.Output;
with PolyORB_HI_Generated.Transport;

with Python;   use Python;

with POHICDRIVER_BLUETOOTH;


package body PolyORB_HI_Drivers_Client_BLE is

   pragma SPARK_Mode (Off);

   pragma Suppress (Elaboration_Check, PolyORB_HI_Generated.Transport);
   --  We do not want a pragma Elaborate_All to be implicitely
   --  generated for Transport.

   pragma Linker_Options ("-lpython3.5m");

   package AS renames Ada.Streams;

   use Ada.Real_Time;
   use Interfaces;
   use System;
   use PolyORB_HI.Messages;
   use PolyORB_HI.Utils;
   use PolyORB_HI.Output;
   
   use POHICDRIVER_BLUETOOTH;

   type Bluetooth_Conf_T_Acc is access all POHICDRIVER_Bluetooth.Bluetooth_Conf_T;
   function To_Bluetooth_Conf_T_Acc is new Ada.Unchecked_Conversion
                                               (System.Address, Bluetooth_Conf_T_Acc);

   type State_T is (NONE, CONNECTED, DISCONNECTED);
   
   type Node_Record is record
      Bluetooth_Config : Bluetooth_Conf_T;
      --  Updated with a positive value for Nodes corresponding to Crazyflies
      --  Map nodes with indices in the python list of Cflies
      Cf_ID : Integer := -1;
      State : State_T := NONE;
   end record;

   Nodes : array (Node_Type) of Node_Record;

   subtype AS_One_Element_Stream is AS.Stream_Element_Array (1 .. 1);
   subtype AS_Message_Length_Stream is AS.Stream_Element_Array
     (1 .. Message_Length_Size);
   subtype Message_Length_Stream is Stream_Element_Array
     (1 .. Message_Length_Size);
   subtype AS_Full_Stream is AS.Stream_Element_Array (1 .. PDU_Size);
   subtype Full_Stream is Stream_Element_Array (1 .. PDU_Size);

   function To_PO_HI_Message_Length_Stream is new Ada.Unchecked_Conversion
     (AS_Message_Length_Stream, Message_Length_Stream);
   function To_PO_HI_Full_Stream is new Ada.Unchecked_Conversion
     (AS_Full_Stream, Full_Stream);
   
   
   Crazyradio_ID : constant Natural := 0;
   --  Configuration : 1 PC and N drones => N + 1 Nodes
   --  Node_Type enumerates all the nodes so 'Pos goes from 0 to N
   NB_DRONES : constant Natural := Node_Type'Pos (Node_Type'Last);
   Dead_Links : Natural := 0;
   
   
   -----------------------------------
   --  Get_URI_From_Bluetooth_Conf  --
   -----------------------------------
   function Get_URI_From_Bluetooth_Conf (BLE_Conf : Bluetooth_Conf_T) return URI is
      Node_URI : URI;
   begin
      Node_URI.Radio_ID := Crazyradio_ID;
      Node_URI.Channel  := Natural (BLE_Conf.channel);
      Node_URI.Datarate := BLE_Conf.datarate;
      return Node_URI;
   end Get_URI_From_Bluetooth_Conf;


   ----------------
   -- Initialize --
   ----------------

   procedure Initialize (Name_Table : PolyORB_HI.Utils.Naming_Table_Type) is
      URIs : URIs_List (1 .. Name_Table'Length - 1);
      Node_URI : URI;
      Idx  : Integer := 1;

   begin
      
      Node_URI.Radio_ID := Crazyradio_ID;
      
      -------------------------------
      -- Getting the configuration --
      -------------------------------
      for J in Name_Table'Range loop
         if Name_Table (J).Variable = System.Null_Address then
            --  The structure of the location information is
            --     "bluetooth CHANNEL DATARATE ADDRESS"

            declare
               S : constant String := PolyORB_HI.Utils.To_String
                                          (Name_Table (J).Location);
               Channel  : Unsigned_8;
               Datarate : Unsigned_8;
               Address  : Address_T;

               First, Last, i : Integer;

            begin
               First := S'First;

               --  First parse the prefix "bluetooth"

               Last := Parse_String (S, First, ' ');

               if S (First .. Last) /= "bluetooth" then
                  raise Program_Error with "Invalid configuration";
               end if;
               
               --  Then, parse the channel

               First := Last + 2;
               Last := Parse_String (S, First, ' ');

               begin
                  Channel := Unsigned_8'Value (S (First .. Last));
               exception
                  when others =>
                     raise Program_Error with "Wrong channel" & S (First .. Last);
               end;
               if Channel > 125 then
                  raise Program_Error with "Wrong channel" & S (First .. Last);
               end if;

               --  Then, parse the data rate

               First := Last + 2;
               Last := Parse_String (S, First, ' ');
               
               if S (First .. Last) = "250K" then
                  Datarate :=  0;
               elsif S (First .. Last) = "1M" then
                  Datarate :=  1;
               elsif S (First .. Last) = "2M" then
                  Datarate :=  2;    
               else
                  raise Program_Error with "Wrong data rate: " & S (First .. Last);
               end if;

               --  Finally, parse the address

               First := Last + 2;
               Last := Parse_String (S, First, ' ');

               --  The address should be 5 bytes in hexadecimal
               if Last - First /= 9 then
                  raise Program_Error with "Wrong address: " & S (First .. Last);
               end if;
               
               i := First;
               while i < Last loop
                  Address.Data (i) := Unsigned_8'Value (S (i .. i + 2));
                  i := i + 2; 
               end loop;    

               Nodes (J).Bluetooth_Config.channel  := Bluetooth_Conf_T_channel (Channel);
               Nodes (J).Bluetooth_Config.datarate := Datarate_T'Val (Datarate);
               Nodes (J).Bluetooth_Config.address  := Address;
               
            end;
         else
            --  We got an ASN.1 configuration variable, use it
            --  directly.
            --  Use_ASN1 := True;
            Nodes (J).Bluetooth_Config := To_Bluetooth_Conf_T_Acc
              (Name_Table (J).Variable).all;

         end if;
      end loop;
      
      --  If the node is different from this one, construct its URI, 
      --  add it to the list, and give the Node a corresponding Cf_ID  
      for N in Node_Type'Range loop
         if N /= My_Node then
            URIs (Idx) := Get_URI_From_Bluetooth_Conf (Nodes (N).Bluetooth_Config);
            Nodes (N).Cf_ID := Idx - 1; 
            Idx := Idx + 1;
         end if;
      end loop;

      --  Initialize Python Crazyflie Library, 
      --  Initialize Crazyflies from the list of URIs
      --  and try to connect to them
      Calls.Call_Cflib_Init (URIs);

      for N in Node_Type'Range loop
         if N /= My_Node then
            Nodes (N).State := CONNECTED;
         end if;
      end loop;

   exception
      when E : others =>
         System.IO.Put_Line ("Exception "
                             & Ada.Exceptions.Exception_Name (E));
         System.IO.Put_Line ("Message  "
                             & Ada.Exceptions.Exception_Message (E));

   end Initialize;
   

   -------------
   -- Receive --
   -------------

   procedure Receive is
      use type AS.Stream_Element_Offset;

      --  SEL : AS_Message_Length_Stream;
      SEA : AS_Full_Stream;
      SEO : AS.Stream_Element_Offset;
   begin

      Main_Loop :
      loop
         for N in Node_Type'Range loop
            if N /= My_Node and Nodes (N).State = CONNECTED then
               --  Receive message from Crazyflie Cf_ID
               Calls.Call_Receive_Packet_Data (Nodes (N).Cf_ID, SEA, SEO);
  
               --  The packet contains a PolyORB message
               if SEO > SEA'First then
                  --  Deliver to the peer handler
                  --  pragma Debug (Verbose, Put_Line ("Delivering message"));
                  System.IO.Put_Line ("Delivering message");
                  PolyORB_HI_Generated.Transport.Deliver
                          (Corresponding_Entity
                             (Unsigned_8 (SEA (Message_Length_Size + 1))),
                           To_PO_HI_Full_Stream (SEA)
                           (1 .. Stream_Element_Offset (SEO)));
                  
               --  The connection with this Crazyflie has been lost
               elsif SEO = SEA'First - 1 then
                   System.IO.Put_Line ("Connection lost with Crazyflie " & Integer'Image (Nodes (N).Cf_ID));
                   Nodes (N).State := DISCONNECTED;
                   Dead_Links := Dead_Links + 1;
                   if Dead_Links = NB_DRONES then
                      System.IO.Put_Line ("All connections lost, destroy Cflib");
                      Calls.Call_Cflib_Destroy;
                      exit Main_Loop;
                   end if;

               --  Connection alive but no PolyORB message
               else
                   null;
               end if;
               
            end if;
         end loop;

      end loop Main_Loop;

   exception
      when E : others =>
         System.IO.Put_Line ("Exception "
                             & Ada.Exceptions.Exception_Name (E));
         System.IO.Put_Line ("Message  "
                             & Ada.Exceptions.Exception_Message (E));
         Calls.Call_Cflib_Destroy;
   end Receive;

   
   ----------
   -- Send --
   ----------

   function Send
     (Node    : Node_Type;
      Message : Stream_Element_Array;
      Size    : Stream_Element_Offset)
     return Error_Kind
   is
      AS_Size : AS.Stream_Element_Offset := AS.Stream_Element_Offset (Size);

      --  Note: we cannot cast both array types using
      --  Ada.Unchecked_Conversion because they are unconstrained
      --  types. We cannot either use direct casting because component
      --  types are incompatible. The only time efficient manner to do
      --  the casting is to use representation clauses.
      Msg : AS.Stream_Element_Array (1 .. AS_Size);
      pragma Import (Ada, Msg);
      for Msg'Address use Message'Address;

      Has_Succeed : Boolean;

   begin

      --  The message length should be less than 30 bytes to fit in 1 CRTP Packet
      if Integer (Size) > 30 --  CRTP_MAX_DATA_SIZE
      then
         return Error_Kind'(Error_Transport);
      end if;

      Has_Succeed := Calls.Call_Send_Packet_Data (Nodes (Node).Cf_ID, Msg, AS_Size);
      if Has_Succeed then
         return Error_Kind'(Error_None);
      else
         System.IO.Put_Line ("Packet not sent");
         return Error_Kind'(Error_Transport);
      end if;
      
   exception
      when E : others =>
         System.IO.Put_Line ("Exception "
                             & Ada.Exceptions.Exception_Name (E));
         System.IO.Put_Line ("Message  "
                             & Ada.Exceptions.Exception_Message (E));
      return Error_Kind'(Error_Transport);
   end Send;

end PolyORB_HI_Drivers_Client_BLE;
