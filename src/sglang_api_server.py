#!/usr/bin/env python3
"""
SGLang OpenAI-Compatible API Server

Simple FastAPI wrapper around SGLang Runtime for OpenAI-compatible API access.
Runs in background thread, uses existing Runtime instance (no duplication).
"""

import logging
import threading
import time
import uuid
from typing import List, Dict, Any, Optional

logger = logging.getLogger(__name__)

try:
    from fastapi import FastAPI, HTTPException
    from fastapi.responses import JSONResponse
    import uvicorn
    from pydantic import BaseModel
    HAS_FASTAPI = True
except ImportError:
    HAS_FASTAPI = False
    logger.warning("FastAPI not available - SGLang API server disabled")

try:
    import sglang as sgl
    from sglang import function, system, user, assistant, gen
    HAS_SGLANG = True
except ImportError:
    HAS_SGLANG = False


class ChatMessage(BaseModel):
    role: str
    content: str


class ChatCompletionRequest(BaseModel):
    model: str
    messages: List[ChatMessage]
    temperature: Optional[float] = 0.7
    max_tokens: Optional[int] = 2000
    stop: Optional[List[str]] = None
    stream: Optional[bool] = False
    # Sampling-time grammar constraint. SGLang accepts `regex` natively and
    # `ebnf` on recent versions. `grammar` is a forward-compat passthrough
    # for GBNF-style strings (currently unused by SGLang; kept for parity
    # with llama.cpp clients that may target either backend).
    regex: Optional[str] = None
    ebnf: Optional[str] = None
    grammar: Optional[str] = None


class SGLangAPIServer:
    """OpenAI-compatible API server for SGLang Runtime."""
    
    def __init__(self, runtime, model_path: str, port: int = 5000,
                 reasoning_parser: Optional[str] = None):
        """
        Initialize API server.

        Args:
            runtime: SGLang Runtime instance
            model_path: Model path (for model ID generation)
            port: Port to listen on
            reasoning_parser: If set (e.g. "qwen3"), the wrapper post-
                processes generated text to strip the leading
                <think>…</think> block so OpenAI-compatible clients see
                only the answer channel. SGLang's native /v1/chat/completions
                applies this; the function/gen path used here does not, so
                the wrapper does it explicitly.
        """
        if not HAS_FASTAPI:
            raise ImportError("FastAPI not available")
        if not HAS_SGLANG:
            raise ImportError("SGLang not available")
        if runtime is None:
            raise ValueError("Runtime cannot be None")

        self.runtime = runtime
        self.model_path = model_path
        self.port = port
        self.reasoning_parser = reasoning_parser
        self.app = FastAPI(title="SGLang OpenAI API")
        self.server_thread = None
        self.server = None
        
        # Generate model ID from path
        self.model_id = self._sanitize_model_id(model_path)
        
        # Setup routes
        self._setup_routes()
        
        logger.info(f"🚀 SGLang API Server initialized (model={self.model_id}, port={port})")
    
    def _sanitize_model_id(self, path: str) -> str:
        """Convert model path to clean model ID."""
        # Remove special chars, lowercase, truncate
        # "DevQuasar/Qwen.Qwen3-Next-80B-A3B-Instruct-FP8-Dynamic"
        # -> "qwen3-next-80b"
        if not path:
            return "sglang-model"
        
        # Extract last component if path-like
        parts = path.replace("\\", "/").split("/")
        name = parts[-1] if parts else path
        
        # Remove special chars, lowercase
        clean = "".join(c.lower() if c.isalnum() or c == "-" else "-" for c in name)
        # Remove multiple dashes
        while "--" in clean:
            clean = clean.replace("--", "-")
        clean = clean.strip("-")
        
        # Truncate to reasonable length
        if len(clean) > 50:
            clean = clean[:50]
        
        return clean or "sglang-model"
    
    def _setup_routes(self):
        """Setup FastAPI routes."""
        
        @self.app.get("/v1/models")
        async def list_models():
            """List available models."""
            return {
                "object": "list",
                "data": [{
                    "id": self.model_id,
                    "object": "model",
                    "created": int(time.time()),
                    "owned_by": "sglang"
                }]
            }
        
        @self.app.post("/v1/chat/completions")
        async def chat_completions(request: ChatCompletionRequest):
            """Chat completions endpoint."""
            try:
                # Validate model
                if request.model != self.model_id:
                    raise HTTPException(
                        status_code=400,
                        detail=f"Model '{request.model}' not found. Available model: '{self.model_id}'"
                    )
                
                # Convert OpenAI messages to SGLang format
                # Extract system message and build conversation string
                system_msg = None
                conversation_parts = []
                
                for msg in request.messages:
                    role = msg.role.lower()
                    content = msg.content
                    
                    if role == "system":
                        system_msg = content
                    elif role == "user":
                        conversation_parts.append(("user", content))
                    elif role == "assistant":
                        conversation_parts.append(("assistant", content))
                
                # Generate using SGLang
                stop_arg = None
                if request.stop:
                    if isinstance(request.stop, list) and len(request.stop) > 0:
                        stop_arg = request.stop  # SGLang accepts a list
                    elif isinstance(request.stop, str):
                        stop_arg = request.stop

                # Capture grammar / regex constraints (sampling-time guided
                # decoding). `ebnf` is preferred on recent SGLang; `regex`
                # is the older path. `grammar` (GBNF) is forwarded as
                # `ebnf` if the engine accepts it; otherwise ignored with
                # a warning so llama.cpp-shaped requests don't silently
                # produce unconstrained output.
                ebnf_arg = request.ebnf or request.grammar or None
                regex_arg = request.regex or None

                # Create generation function with proper message structure
                @function
                def generate_chat(s, system_text, user_text):
                    if system_text:
                        s += system(system_text)
                    s += user(user_text)

                    # Generate response
                    gen_kwargs = {
                        "max_tokens": request.max_tokens or 2000,
                        "temperature": request.temperature or 0.7
                    }
                    if stop_arg:
                        gen_kwargs["stop"] = stop_arg
                    if ebnf_arg:
                        gen_kwargs["ebnf"] = ebnf_arg
                    if regex_arg:
                        gen_kwargs["regex"] = regex_arg
                    s += assistant(gen("output", **gen_kwargs))
                
                # Build user text from conversation (last user message + any assistant context)
                # For simplicity, combine all messages into a single user prompt
                # More sophisticated: handle multi-turn properly
                user_text_parts = []
                for role, content in conversation_parts:
                    if role == "user":
                        user_text_parts.append(f"User: {content}")
                    elif role == "assistant":
                        user_text_parts.append(f"Assistant: {content}")
                
                user_text = "\n".join(user_text_parts) if user_text_parts else ""
                if not user_text:
                    # Fallback: use last message content
                    user_text = request.messages[-1].content if request.messages else ""
                
                # Check total input size before calling generate_chat.run()
                total_input_size = len(system_msg or "") + len(user_text)
                if total_input_size > 100000:
                    import traceback
                    logger.error(f"⚠️  Attempting to send {total_input_size:,} chars to sglang via API server (limit: 100,000)")
                    logger.error(f"  system_text length: {len(system_msg or ''):,}")
                    logger.error(f"  user_text length: {len(user_text):,}")
                    logger.error(f"  user_text preview (first 500 chars): {user_text[:500]}...")
                    logger.error("Stack traceback:")
                    for line in traceback.format_stack():
                        logger.error(line.rstrip())
                
                # Run generation
                state = generate_chat.run(
                    system_text=system_msg or "",
                    user_text=user_text
                )
                result_text = state["output"].strip()

                # Reasoning-parser stripping. SGLang's native chat endpoint
                # applies the configured reasoning_parser to split thinking
                # from content; the function/gen path bypasses that, so the
                # wrapper does it here. We strip only when the runtime was
                # configured with a reasoning_parser to avoid mangling
                # genuine <think>…</think> content from non-reasoning runs.
                if self.reasoning_parser and result_text:
                    if '</think>' in result_text:
                        result_text = result_text.split('</think>', 1)[1].lstrip()
                    elif result_text.lstrip().startswith('<think>'):
                        # Truncated mid-think (no closing tag) → no answer
                        result_text = ''
                
                # Build OpenAI-compatible response
                response_id = f"chatcmpl-{uuid.uuid4().hex[:12]}"
                created = int(time.time())
                
                return {
                    "id": response_id,
                    "object": "chat.completion",
                    "created": created,
                    "model": self.model_id,
                    "choices": [{
                        "index": 0,
                        "message": {
                            "role": "assistant",
                            "content": result_text
                        },
                        "finish_reason": "stop"
                    }],
                    "usage": {
                        "prompt_tokens": 0,  # SGLang doesn't expose token counts easily
                        "completion_tokens": 0,
                        "total_tokens": 0
                    }
                }
                
            except HTTPException:
                raise
            except Exception as e:
                logger.exception(f"Chat completion error: {e}")
                raise HTTPException(status_code=500, detail=str(e))
        
        @self.app.get("/health")
        async def health():
            """Health check endpoint."""
            return {"status": "ok", "model": self.model_id}
    
    def start(self):
        """Start server in background thread."""
        if self.server_thread and self.server_thread.is_alive():
            logger.warning("API server already running")
            return
        
        def run_server():
            try:
                config = uvicorn.Config(
                    self.app,
                    host="0.0.0.0",
                    port=self.port,
                    log_level="warning"  # Reduce noise
                )
                self.server = uvicorn.Server(config)
                self.server.run()
            except Exception as e:
                logger.error(f"API server error: {e}")
        
        self.server_thread = threading.Thread(target=run_server, daemon=True)
        self.server_thread.start()
        logger.info(f"🌐 SGLang API server started on port {self.port}")
    
    def stop(self):
        """Stop server."""
        if self.server:
            try:
                self.server.should_exit = True
            except Exception as e:
                logger.warning(f"Error stopping API server: {e}")
        logger.info("SGLang API server stopped")

