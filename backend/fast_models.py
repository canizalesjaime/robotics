from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], 
    allow_methods=["*"],
    allow_headers=["*"],
)

class Command(BaseModel):
    action: str

class Speed(BaseModel):
    speed: int

class ArmAngles(BaseModel):
    base: int
    shoulder: int
    elbow: int
    gripper: int
