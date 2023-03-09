import logging

import asyncio
from aioconsole.console import AsynchronousConsole

# We use aioconsole to provide development console.
CONSOLE_ADDR = '0.0.0.0'
CONSOLE_PORT = 13501


# class AsynchronousConsole(AsynchronousConsole):
#     # Note: this relies on the creating code assigning a "log" attribute.
#     async def raw_input(self, prompt=""):
#         result = await self.ainput(prompt)
#         # self.log.debug('%r', result)
#         return result



class CLI:
    '''Custom aioconsole thing that tracks clients so we can
    actually force them closed when we're trying to shut down.
    Without this, if there's an open console the whole process
    stalls at shutdown waiting for the last socket to close.
    '''

    def __init__(self, locals):
        self.locals = locals
        self.server = None
        self.clients = {}
        self.log = logging.getLogger('CLI')
        self.count = 0


    async def handle_client(self, reader, writer):
        self.count += 1
        count = self.count
        self.log.info('#%s opened', count)

        # clocals = dict(self.locals) if self.locals is not None else None
        clocals = self.locals       # persist changes and share with all clients

        client = AsynchronousConsole(
            streams=(reader, writer),
            locals=clocals,
            filename='<console>',
            )

        client.log = logging.getLogger(f'console.{count}')
        # TODO: add cleanup to remove this from the loggers, though it's
        # really not that useful since it's an incredibly minor memory
        # leak and only during development.

        try:
            self.clients[count] = reader    # to allow clean shutdown

            await client.interact(stop=False, handle_sigint=False)

        except EOFError:
            pass

        except Exception as ex:
            self.log.exception('#%s: %s', count, ex)

        finally:
            writer.close()

            self.clients.pop(count, None)
            self.log.info('#%s closed', count)


    def close(self):
        self.server.close()
        for count, client in self.clients.items():
            self.log.debug('#%s closing', count)
            client.feed_eof()


    async def wait_closed(self):
        await self.server.wait_closed()


    @classmethod
    async def start_server(cls, locals, hostport=None):
        cli = cls(locals)
        host, port = hostport.split(':') if hostport else (CONSOLE_ADDR, CONSOLE_PORT)

        cli.server = await asyncio.start_server(cli.handle_client, host, port,
            reuse_address=True) # reuse_port=True)
        return cli
