#ifndef __HCI_APIH__
#define __HCI_APIH__

#include "HCI_constants.h"  /* HCI data type constants */
#include "bt_types.h"

#define HEADER_SIZE 	2

/* This structure contains all the necessary values associated with an
* active bluetooth connection. */
typedef struct _tagHCI_Bt_inst_t
{
   BD_ADDR_t   BD_ADDR;
   Link_Key_t  Link_Key;
   LAP_t       LAP;
} HCI_Bt_Inst_t;

#define HCI_BT_INST_SIZE                                (sizeof(HCI_Bt_Inst_t))

/* HCI Event API Types.                                              */
typedef enum
{
   etInquiry_Complete_Event,
   etInquiry_Result_Event,
   etConnection_Complete_Event,
   etConnection_Request_Event,
   etDisconnection_Complete_Event,
   etAuthentication_Complete_Event,
   etRemote_Name_Request_Complete_Event,
   etEncryption_Change_Event,
   etChange_Connection_Link_Key_Complete_Event,
   etMaster_Link_Key_Complete_Event,
   etRead_Remote_Supported_Features_Complete_Event,
   etRead_Remote_Version_Information_Complete_Event,
   etQoS_Setup_Complete_Event,
   etCommand_Complete_Event,
	 etHardware_Error_Event,
   etFlush_Occurred_Event,
   etRole_Change_Event,
   etNumber_Of_Completed_Packets_Event,
   etMode_Change_Event,
   etReturn_Link_Keys_Event,
   etPIN_Code_Request_Event,
   etLink_Key_Request_Event,
   etLink_Key_Notification_Event,
   etLoopback_Command_Event,
   etData_Buffer_Overflow_Event,
   etMax_Slots_Change_Event,
   etRead_Clock_Offset_Complete_Event,
   etConnection_Packet_Type_Changed_Event,
   etQoS_Violation_Event,
   etPage_Scan_Mode_Change_Event,
   etPage_Scan_Repetition_Mode_Change_Event,
   etBluetooth_Logo_Testing_Event,
   etVendor_Specific_Debug_Event,
   etDevice_Reset_Event,
   etFlow_Specification_Complete_Event,
   etInquiry_Result_With_RSSI_Event,
   etRead_Remote_Extended_Features_Complete_Event,
   etSynchronous_Connection_Complete_Event,
   etSynchronous_Connection_Changed_Event,
   etSniff_Subrating_Event,
   etExtended_Inquiry_Result_Event,
   etEncryption_Key_Refresh_Complete_Event,
   etIO_Capability_Request_Event,
   etIO_Capability_Response_Event,
   etUser_Confirmation_Request_Event,
   etUser_Passkey_Request_Event,
   etRemote_OOB_Data_Request_Event,
   etSimple_Pairing_Complete_Event,
   etLink_Supervision_Timeout_Changed_Event,
   etEnhanced_Flush_Complete_Event,
   etUser_Passkey_Notification_Event,
   etKeypress_Notification_Event,
   etRemote_Host_Supported_Features_Notification_Event,
   etPhysical_Link_Complete_Event,
   etChannel_Selected_Event,
   etDisconnection_Physical_Link_Complete_Event,
   etPhysical_Link_Loss_Early_Warning_Event,
   etPhysical_Link_Recovery_Event,
   etLogical_Link_Complete_Event,
   etDisconnection_Logical_Link_Complete_Event,
   etFlow_Spec_Modify_Complete_Event,
   etNumber_Of_Completed_Data_Blocks_Event,
   etShort_Range_Mode_Change_Complete_Event,
   etAMP_Status_Change_Event,
   etAMP_Start_Test_Event,
   etAMP_Test_End_Event,
   etAMP_Receiver_Report_Event,
   etLE_Meta_Event,
   etPlatform_Specific_Event
} HCI_Event_Type_t;

typedef struct _tagHCI_Event_Packet_Header_t
{
   HCI_PacketType_t HCIPacketType;
	 uint8_t					HCIEventCode;
   uint8_t		      HCIPacketLength;
   uint8_t		      *HCIPacketData;
} HCI_Event_Packet_Header_t;

#define HCI_EVENT_PACKET_HEADER_SIZE                    (sizeof(HCI_Event_Packet_Header_t))


#define HCI_SET_MWS_TRANSPORT_LAYER_RESPONSE_PARAM_SIZE 1


typedef struct _tagHCI_Stored_Link_Key_Info_t
{
   BD_ADDR_t  BD_ADDR;
   Link_Key_t Link_Key;
} HCI_Stored_Link_Key_Info_t;

#define HCI_STORED_LINK_KEY_INFO_SIZE                   (sizeof(HCI_Stored_Link_Key_Info_t))



typedef struct _tagHCI_Inquiry_Complete_Event_Data_t
{
   uint8_t Status;
   uint8_t Num_Responses;
} HCI_Inquiry_Complete_Event_Data_t;

#define HCI_INQUIRY_COMPLETE_EVENT_DATA_SIZE            (sizeof(HCI_Inquiry_Complete_Event_Data_t))

   /* The following structure represents the data that is returned in   */
   /* a HCI Inquiry Result Event.  This structure represents an         */
   /* individual entry, and the HCI Inquiry Result Event structure      */
   /* contains an array of these structures (based upon the number of   */
   /* responses).                                                       */
typedef struct _tagHCI_Inquiry_Result_Data_t
{
   BD_ADDR_t          BD_ADDR;
   uint8_t            Page_Scan_Repetition_Mode;
   uint8_t            Page_Scan_Period_Mode;
   uint8_t            Page_Scan_Mode;
   Class_of_Device_t  Class_of_Device;
   uint16_t           Clock_Offset;
} HCI_Inquiry_Result_Data_t;

#define HCI_INQUIRY_RESULT_DATA_SIZE                    (sizeof(HCI_Inquiry_Result_Data_t))

   /* The following structure represents the Data that is associated    */
   /* with the HCI Inquiry Result Event.  The HCI_Inquiry_Result_Data   */
   /* member represents a variable array that can contain 0 or more     */
   /* entries.  The Number of Entries in this array is given by the     */
   /* Num_Responses member.  A utility MACRO is provided to aid in the  */
   /* calculation of the Number of Bytes that are occupied by the       */
   /* structure given a specified number of Responses.                  */

typedef struct _tagHCI_Inquiry_Result_Event_Data_t
{
   uint8_t                    Num_Responses;
   HCI_Inquiry_Result_Data_t  HCI_Inquiry_Result_Data[1];
} HCI_Inquiry_Result_Event_Data_t;

   /* The following MACRO is a utility MACRO that exists to aid code    */
   /* readability to Determine the size (in Bytes) of an                */
   /* HCI Inquiry Result Event Data Structure based upon the number of  */
   /* HCI Inquiry Result Entries associated with the Event.  The first  */
   /* parameter to this MACRO is the number of HCI Inquiry Result       */
   /* Entries.                                                          */
#define HCI_INQUIRY_RESULT_EVENT_DATA_SIZE(_x)          ((sizeof(HCI_Inquiry_Result_Event_Data_t) - sizeof(HCI_Inquiry_Result_Data_t)) + (((Byte_t)(_x))*sizeof(HCI_Inquiry_Result_Data_t)))



   /* The following structure represents the container structure for    */
   /* Holding all HCI Event Data Data.                                  */
typedef struct _tagHCI_Event_Data_t
{
   HCI_Event_Type_t Event_Data_Type;
   uint8_t          Event_Data_Size;
   union
   {
      HCI_Inquiry_Complete_Event_Data_t                            *HCI_Inquiry_Complete_Event_Data;
      HCI_Inquiry_Result_Event_Data_t                              *HCI_Inquiry_Result_Event_Data;
      /*HCI_Connection_Complete_Event_Data_t                         *HCI_Connection_Complete_Event_Data;
      HCI_Connection_Request_Event_Data_t                          *HCI_Connection_Request_Event_Data;
      HCI_Disconnection_Complete_Event_Data_t                      *HCI_Disconnection_Complete_Event_Data;
      HCI_Authentication_Complete_Event_Data_t                     *HCI_Authentication_Complete_Event_Data;
      HCI_Remote_Name_Request_Complete_Event_Data_t                *HCI_Remote_Name_Request_Complete_Event_Data;
      HCI_Encryption_Change_Event_Data_t                           *HCI_Encryption_Change_Event_Data;
      HCI_Change_Connection_Link_Key_Complete_Event_Data_t         *HCI_Change_Connection_Link_Key_Complete_Event_Data;
      HCI_Master_Link_Key_Complete_Event_Data_t                    *HCI_Master_Link_Key_Complete_Event_Data;
      HCI_Read_Remote_Supported_Features_Complete_Event_Data_t     *HCI_Read_Remote_Supported_Features_Complete_Event_Data;
      HCI_Read_Remote_Version_Information_Complete_Event_Data_t    *HCI_Read_Remote_Version_Information_Complete_Event_Data;
      HCI_QoS_Setup_Complete_Event_Data_t                          *HCI_QoS_Setup_Complete_Event_Data;
      HCI_Hardware_Error_Event_Data_t                              *HCI_Hardware_Error_Event_Data;
      HCI_Flush_Occurred_Event_Data_t                              *HCI_Flush_Occurred_Event_Data;
      HCI_Role_Change_Event_Data_t                                 *HCI_Role_Change_Event_Data;
      HCI_Number_Of_Completed_Packets_Event_Data_t                 *HCI_Number_Of_Completed_Packets_Event_Data;
      HCI_Mode_Change_Event_Data_t                                 *HCI_Mode_Change_Event_Data;
      HCI_Return_Link_Keys_Event_Data_t                            *HCI_Return_Link_Keys_Event_Data;
      HCI_PIN_Code_Request_Event_Data_t                            *HCI_PIN_Code_Request_Event_Data;
      HCI_Link_Key_Request_Event_Data_t                            *HCI_Link_Key_Request_Event_Data;
      HCI_Link_Key_Notification_Event_Data_t                       *HCI_Link_Key_Notification_Event_Data;
      HCI_Loopback_Command_Event_Data_t                            *HCI_Loopback_Command_Event_Data;
      HCI_Data_Buffer_Overflow_Event_Data_t                        *HCI_Data_Buffer_Overflow_Event_Data;
      HCI_Max_Slots_Change_Event_Data_t                            *HCI_Max_Slots_Change_Event_Data;
      HCI_Read_Clock_Offset_Complete_Event_Data_t                  *HCI_Read_Clock_Offset_Complete_Event_Data;
      HCI_Connection_Packet_Type_Changed_Event_Data_t              *HCI_Connection_Packet_Type_Changed_Event_Data;
      HCI_QoS_Violation_Event_Data_t                               *HCI_QoS_Violation_Event_Data;
      HCI_Command_Complete_Event_Data_t														 *HCI_Command_Complete_Event_Data;
			HCI_Page_Scan_Repetition_Mode_Change_Event_Data_t            *HCI_Page_Scan_Repetition_Mode_Change_Event_Data;
      HCI_Page_Scan_Mode_Change_Event_Data_t                       *HCI_Page_Scan_Mode_Change_Event_Data;
      HCI_Flow_Specification_Complete_Event_Data_t                 *HCI_Flow_Specification_Complete_Event_Data;
      HCI_Inquiry_Result_With_RSSI_Event_Data_t                    *HCI_Inquiry_Result_With_RSSI_Event_Data;
      HCI_Read_Remote_Extended_Features_Complete_Event_Data_t      *HCI_Read_Remote_Extended_Features_Complete_Event_Data;
      HCI_Synchronous_Connection_Complete_Event_Data_t             *HCI_Synchronous_Connection_Complete_Event_Data;
      HCI_Synchronous_Connection_Changed_Event_Data_t              *HCI_Synchronous_Connection_Changed_Event_Data;
      HCI_Sniff_Subrating_Event_Data_t                             *HCI_Sniff_Subrating_Event_Data;
      HCI_Extended_Inquiry_Result_Event_Data_t                     *HCI_Extended_Inquiry_Result_Event_Data;
      HCI_Encryption_Key_Refresh_Complete_Event_Data_t             *HCI_Encryption_Key_Refresh_Complete_Event_Data;
      HCI_IO_Capability_Request_Event_Data_t                       *HCI_IO_Capability_Request_Event_Data;
      HCI_IO_Capability_Response_Event_Data_t                      *HCI_IO_Capability_Response_Event_Data;
      HCI_User_Confirmation_Request_Event_Data_t                   *HCI_User_Confirmation_Request_Event_Data;
      HCI_User_Passkey_Request_Event_Data_t                        *HCI_User_Passkey_Request_Event_Data;
      HCI_Remote_OOB_Data_Request_Event_Data_t                     *HCI_Remote_OOB_Data_Request_Event_Data;
      HCI_Simple_Pairing_Complete_Event_Data_t                     *HCI_Simple_Pairing_Complete_Event_Data;
      HCI_Link_Supervision_Timeout_Changed_Event_Data_t            *HCI_Link_Supervision_Timeout_Changed_Event_Data;
      HCI_Enhanced_Flush_Complete_Event_Data_t                     *HCI_Enhanced_Flush_Complete_Event_Data;
      HCI_User_Passkey_Notification_Event_Data_t                   *HCI_User_Passkey_Notification_Event_Data;
      HCI_Keypress_Notification_Event_Data_t                       *HCI_Keypress_Notification_Event_Data;
      HCI_Remote_Host_Supported_Features_Notification_Event_Data_t *HCI_Remote_Host_Supported_Features_Notification_Event_Data;
      HCI_Physical_Link_Complete_Event_Data_t                      *HCI_Physical_Link_Complete_Event_Data;
      HCI_Channel_Selected_Event_Data_t                            *HCI_Channel_Selected_Event_Data;
      HCI_Disconnection_Physical_Link_Complete_Event_Data_t        *HCI_Disconnection_Physical_Link_Complete_Event_Data;
      HCI_Physical_Link_Loss_Early_Warning_Event_Data_t            *HCI_Physical_Link_Loss_Early_Warning_Event_Data;
      HCI_Physical_Link_Recovery_Event_Data_t                      *HCI_Physical_Link_Recovery_Event_Data;
      HCI_Logical_Link_Complete_Event_Data_t                       *HCI_Logical_Link_Complete_Event_Data;
      HCI_Disconnection_Logical_Link_Complete_Event_Data_t         *HCI_Disconnection_Logical_Link_Complete_Event_Data;
      HCI_Flow_Spec_Modify_Complete_Event_Data_t                   *HCI_Flow_Spec_Modify_Complete_Event_Data;
      HCI_Number_Of_Completed_Data_Blocks_Event_Data_t             *HCI_Number_Of_Completed_Data_Blocks_Event_Data;
      HCI_Short_Range_Mode_Change_Complete_Event_Data_t            *HCI_Short_Range_Mode_Change_Complete_Event_Data;
      HCI_AMP_Status_Change_Event_Data_t                           *HCI_AMP_Status_Change_Event_Data;
      HCI_AMP_Start_Test_Event_Data_t                              *HCI_AMP_Start_Test_Event_Data;
      HCI_AMP_Test_End_Event_Data_t                                *HCI_AMP_Test_End_Event_Data;
      HCI_AMP_Receiver_Report_Event_Data_t                         *HCI_AMP_Receiver_Report_Event_Data;
      HCI_LE_Meta_Event_Data_t                                     *HCI_LE_Meta_Event_Data;
      HCI_Platform_Specific_Event_Data_t                           *HCI_Platform_Specific_Event_Data;
      void                                                         *HCI_Unknown_Event_Data;*/
   } Event_Data;
} HCI_Event_Data_t;

#define HCI_EVENT_DATA_SIZE                                                (sizeof(HCI_Event_Data_t))



#endif // __HCI_API__ 