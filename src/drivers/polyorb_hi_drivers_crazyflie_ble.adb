------------------------------------------------------------------------------
--                                                                          --
--                          PolyORB HI COMPONENTS                           --
--                                                                          --
--     P O L Y O R B _ H I _ D R I V E R S _ C R A Z Y F L I E _  B L E     --
--                                                                          --
--                                 S p e c                                  --
--                                                                          --
--                   Copyright (C) 2012-2015 ESA & ISAE.                    --
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

pragma Style_Checks (off);

with Types;               use Types;

with Crazyflie_System;    use Crazyflie_System;
with Crazyflie_Config;    use Crazyflie_Config;
with Radiolink;           use Radiolink;
with CRTP;                use CRTP;
with Syslink;             use Syslink;
with UART_Syslink;        use UART_Syslink;
with EXT_UART;            use EXT_UART;
with Tasks;               use Tasks;
with CRTP_Service;        use CRTP_Service;

with Ada.Interrupts;         use Ada.Interrupts;
with Ada.Interrupts.Names;   use Ada.Interrupts.Names;
with Interfaces;
With Ada.Exceptions;
with Ada.Unchecked_Conversion;
with Ada.Streams;

with PolyORB_HI.Output;
with PolyORB_HI.Messages;
with PolyORB_HI_Generated.Transport;

with System; use System;

with POHICDRIVER_BLUETOOTH;


package body PolyORB_HI_Drivers_Crazyflie_BLE is

   --  TODO : add BLE_Conf_T_Acc and Nodes

   pragma Suppress (Elaboration_Check, PolyORB_HI_Generated.Transport);
   --  We do not want a pragma Elaborate_All to be implicitely
   --  generated for Transport.

   package AS renames Ada.Streams;

   use Interfaces;
   use PolyORB_HI.Messages;
   use PolyORB_HI.Utils;
   use PolyORB_HI.Output;

   use POHICDRIVER_BLUETOOTH;

   type Bluetooth_Conf_T_Acc is access all POHICDRIVER_Bluetooth.Bluetooth_Conf_T;
   function To_Bluetooth_Conf_T_Acc is new Ada.Unchecked_Conversion
     (System.Address, Bluetooth_Conf_T_Acc);

   type Node_Record is record
      Bluetooth_Config : Bluetooth_Conf_T;
   end record;

   Nodes : array (Node_Type) of Node_Record;

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

   Connected_To_Radio : Boolean := False;
   Dropped_Packets    : T_Uint8 := 0;
   Enqueued_Packets   : T_Uint8 := 0;

   ----------------
   -- Initialize --
   ----------------

   procedure Initialize (Name_Table : PolyORB_HI.Utils.Naming_Table_Type) is
      --  Use_ASN1 : Boolean := False;
      My_Channel  : T_Uint8;
      My_Datarate : T_Uint8;
      My_Address  : T_Uint64;

      function Address_To_Uint64 is new Ada.Unchecked_Conversion
        (Address_T_array, T_Uint64);
   begin
      Initialize_EXT_UART;
      System_Init;
      Connected_To_Radio := False;
      if System_Self_Test then
         null; --  Send_To_UART("[BLE] initialized");
      end if;

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
               Channel  : T_Uint8;
               Datarate : T_Uint8;
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
                  Channel := T_Uint8'Value (S (First .. Last));
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
                  Datarate :=  RADIO_RATE'Pos (RADIO_RATE_250K);
               elsif S (First .. Last) = "1M" then
                  Datarate :=  RADIO_RATE'Pos (RADIO_RATE_1M);
               elsif S (First .. Last) = "2M" then
                  Datarate :=  RADIO_RATE'Pos (RADIO_RATE_2M);    
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

      My_Channel  := T_Uint8 (Nodes (My_Node).Bluetooth_Config.channel);
      My_Datarate := Datarate_T'Pos (Nodes (My_Node).Bluetooth_Config.datarate);
      My_Address  := Address_To_Uint64 (Nodes (My_Node).Bluetooth_Config.address.Data);

      --  Update radio configuration if necessary
      if My_Channel /= RADIO_CHANNEL then
         Radiolink_Set_Channel (My_Channel);
      end if;

     if My_Datarate /= RADIO_DATARATE then
         Radiolink_Set_Data_Rate (My_Datarate);
     end if;

     if My_Address /= RADIO_ADDRESS then
        Radiolink_Set_Address (My_Address);
     end if;

     Send_To_UART ("[BLE] Configured " 
                   & T_Uint8'Image (My_Channel) & " "
                   & Datarate_T'Image (Nodes (My_Node).Bluetooth_Config.datarate) & " "
                   & T_Uint64'Image (My_Address));

   end Initialize;


   --------------------------
   -- CRTP_Service_Handler --
   --------------------------

   procedure CRTP_Service_Handler (Packet : CRTP_Packet)
   is
      Command     : CRTP_Service_Command;
      Tx_Packet   : CRTP_Packet := Packet;
      Has_Succeed : Boolean;

      ------------------------------------------
      -- CRTP_Channel_To_CRTP_Service_Command --
      ------------------------------------------

      function CRTP_Channel_To_CRTP_Service_Command is
        new Ada.Unchecked_Conversion (CRTP_Channel, CRTP_Service_Command);
   begin
      --  Send_To_UART("[CRTP_Service] Handling packet");

      Command := CRTP_Channel_To_CRTP_Service_Command (Packet.Channel);

      case Command is
         when Link_Echo =>
            Has_Succeed := Radiolink_Send_Packet (Tx_Packet);
         when Link_Source =>
            Tx_Packet.Size := CRTP_MAX_DATA_SIZE;
            Has_Succeed := Radiolink_Send_Packet (Tx_Packet);
         when others =>
            --  Null packets
            null;
      end case;
   end CRTP_Service_Handler;


   -------------
   -- Receive --
   -------------

   procedure Receive is
      use type Ada.Streams.Stream_Element_Offset;

      procedure CRTP_Get_T_Uint8_Data is new CRTP_Get_Data (T_Uint8);

      Rx_Packet : CRTP_Packet;
      Rx_Byte : T_Uint8;
      Has_Succeed : Boolean;
      SEA : AS_Full_Stream;
      SEO : AS.Stream_Element_Offset;
   begin

      Main_Loop : loop

         --  Wait until a CRTP Packet is received
         Radiolink_Receive_Packet_Blocking(Rx_Packet);
         --  Send_To_UART("[Receive] CRTP Packet received");

         if Rx_Packet.Port = CRTP_PORT_LINK
         then
            --  Send_To_UART("[Receive] LINK");
            CRTP_Service_Handler (Rx_Packet);

         elsif Rx_Packet.Port = CRTP_PORT_CONSOLE
         then
            --  Send_To_UART("[Receive] Packet received on CONSOLE port");

            --  Extract PolyORB message
            SEO := AS.Stream_Element_Offset (Rx_Packet.Data_2'Length);

            for i in Rx_Packet.Data_2'Range loop
               CRTP_Get_T_Uint8_Data (CRTP_Get_Handler_From_Packet (Rx_Packet),
                                      i, Rx_Byte, Has_Succeed);
               SEA (AS.Stream_Element_Offset (i)) := AS.Stream_Element (Rx_Byte);
            end loop;


            --  Deliver to the peer handler
            begin
               PolyORB_HI_Generated.Transport.Deliver
                 (Corresponding_Entity
                       (Unsigned_8 (SEA (Message_Length_Size + 1))),
                  To_PO_HI_Full_Stream (SEA)
                       (1 .. Stream_Element_Offset (SEO)));
            exception
               when E : others =>
                  null; --  Put_Line (Ada.Exceptions.Exception_Information (E));
            end;

         else
            null;

         end if;

      end loop Main_Loop;
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
      pragma Unreferenced (Node);
      procedure CRTP_Append_T_Uint8_Data is new CRTP_Append_Data (T_Uint8);

      --  We cannot cast both array types using
      --  Ada.Unchecked_Conversion because they are unconstrained
      --  types. We cannot either use direct casting because component
      --  types are incompatible. The only time efficient manner to do
      --  the casting is to use representation clauses.

      Msg : AS.Stream_Element_Array
        (1 .. Ada.Streams.Stream_Element_Offset (Size));
      pragma Import (Ada, Msg);
      for Msg'Address use Message'Address;
      --  Using CRTP protocol
      Packet_Handler : CRTP_Packet_Handler;
      Has_Succeed : Boolean;
      Free_Bytes_In_Packet : Boolean := True;

      Size_Int : constant Integer 
        := Integer (Ada.Streams.Stream_Element_Offset (Size));

   begin

      --  The message length should be less than 30 bytes to fit in 1 CRTP Packet
      if Integer (Size) > CRTP_MAX_DATA_SIZE
      then
         Send_To_UART ("Message too long :" & Integer'Image (Size_Int));
         return Error_Kind'(Error_Transport);
      else
         --  Send_To_UART ("Size :" & Integer'Image (Size_Int));

         --  Use the Console Port to send the message to client
         --  This port is checked by the python client
         Packet_Handler := CRTP_Create_Packet (CRTP_PORT_CONSOLE, 0);

         --  Append message to the CRTP Packet
         for SE of Msg loop
            CRTP_Append_T_Uint8_Data
              (Packet_Handler, T_Uint8 (SE), Free_Bytes_In_Packet);
         end loop;

         --  Send the CRTP Packet by radio
         Has_Succeed := Radiolink_Send_Packet
           (CRTP_Get_Packet_From_Handler (Packet_Handler));
         --  Send_To_UART ("Sent: " & Boolean'Image (Has_Succeed));

         if Connected_To_Radio then
            -- We are connected to a radio
            if Has_Succeed = False then
               Dropped_Packets := Dropped_Packets + 1;
            else
               Dropped_Packets := 0;
            end if;

            --  If too many packets are dropped,
            --  reset the UART talking to the nRF51
            if Dropped_Packets = 10 then
               UART_Syslink_Init;
               Dropped_Packets := 0;
               Send_To_UART ("=== "
                             & "RESET COMMUNICATION"
                             & " ===");
            end if;
         else
            --  We don't want to reset if the communication is not established
            if Has_Succeed then
               Enqueued_Packets := Enqueued_Packets + 1;
            end if;
            --  The size of the Tx queue of Radiolink is 3
            if Enqueued_Packets >= 4 then
               Connected_To_Radio := True;
            end if;    
         end if;

         return Error_Kind'(Error_None);
         --  Note: we have no way to know there was an error here
      end if;
   end;

end PolyORB_HI_Drivers_Crazyflie_BLE;
