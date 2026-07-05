# This file contains the customized messages
# DO NOT MODIFY THIS FILE
# Contact: support@manipulation-net.org

from typing import Literal, Dict, Any, Union, Optional

try:
    from pydantic import BaseModel, TypeAdapter
except Exception as e:
    print(f"Error importing pydantic: {e}")
    print("Please ensure pydantic >=2.0 is installed and properly configured.")
    exit()

HEADER_FMT = "!I"
HEADER_SIZE = 4


class LoginRequest(BaseModel):
    """
    Login request
    """

    type: Literal["login_request"]
    team_unique_code: str
    autonomy_level: int
    connection_test: bool


class LoginResponse(BaseModel):
    """
    Login response
    """

    type: Literal["login_response"]
    success: bool
    message: str
    one_time_code: Optional[str] = None


class TaskRequest(BaseModel):
    """
    Benchmark task request
    """

    type: Literal["task_request"]
    message: str


class TaskResponse(BaseModel):
    """
    Benchmark task response
    """

    type: Literal["task_response"]
    success: bool
    benchmark_name: str
    task_details: Dict[str, Any]
    instruction_enabled: bool
    assistance_allowed: bool
    overlay_enabled: bool
    message: Optional[str] = None


class HashUpdateRequest(BaseModel):
    """
    Hash update request
    """

    type: Literal["hashcode_request"]
    hashcode: str
    order: int


class HashUpdateResponse(BaseModel):
    """
    Hash update response
    """

    type: Literal["hashcode_response"]
    success: bool
    message: str


class ExecutionStatusRequest(BaseModel):
    """
    Execution status request
    """

    type: Literal["status_request"]
    task_name: str
    task_status: str


class ExecutionStatusResponse(BaseModel):
    """
    Execution status response
    """

    type: Literal["status_response"]
    success: bool
    message: str


class PingRequest(BaseModel):
    """
    Ping request
    """

    type: Literal["ping_request"]


class PingResponse(BaseModel):
    """
    Ping response
    """

    type: Literal["ping_response"]
    keyframe_requested: bool
    success: bool


class AssistanceRequest(BaseModel):
    """
    Assistance request
    """

    type: Literal["assistance_request"]
    task_name: str
    assistance_type: str
    assistance_action: str


class AssistanceResponse(BaseModel):
    """
    Assistance response
    """

    type: Literal["assistance_response"]
    success: bool
    message: str


class SubmissionRequest(BaseModel):
    """
    Submission request
    """

    type: Literal["submission_request"]
    finished: bool


class SubmissionResponse(BaseModel):
    """
    Submission response
    """

    type: Literal["submission_response"]
    success: bool
    message: str
    upload_url: str


class ErrorResponse(BaseModel):
    """
    Error response
    """

    success: bool
    message: str


class ShutdownRequest(BaseModel):
    """
    Connection shutdown request
    """

    type: Literal["shutdown_request"]


class InstructionRequest(BaseModel):
    """
    Multimodal prompt request
    """

    type: Literal["instruction_request"]
    task_name: str


class InstructionResponse(BaseModel):
    """
    Multimodal prompt response
    """

    type: Literal["instruction_response"]
    success: bool
    vision: Optional[str] = None
    language: Optional[str] = None


class CameraConfigRequest(BaseModel):
    """
    Camera config request
    """

    type: Literal["camera_config_request"]
    task_config: dict


class CameraConfigResponse(BaseModel):
    """
    Camera config response
    """

    type: Literal["camera_config_response"]
    success: bool
    message: str


ServerResponse = TypeAdapter(
    Union[
        LoginResponse,
        TaskResponse,
        InstructionResponse,
        HashUpdateResponse,
        ExecutionStatusResponse,
        AssistanceResponse,
        SubmissionResponse,
        ErrorResponse,
        PingResponse,
        CameraConfigResponse,
    ]
)
