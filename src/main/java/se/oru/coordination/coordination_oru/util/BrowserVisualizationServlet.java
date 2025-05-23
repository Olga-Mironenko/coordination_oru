package se.oru.coordination.coordination_oru.util;


import org.eclipse.jetty.websocket.servlet.WebSocketServlet;
import org.eclipse.jetty.websocket.servlet.WebSocketServletFactory;

@SuppressWarnings("serial")
public class BrowserVisualizationServlet extends WebSocketServlet
{
    @Override
    public void configure(WebSocketServletFactory factory)
    {
        // Disable permessage-deflate extension
        factory.getExtensionFactory().unregister("permessage-deflate");

        factory.register(BrowserVisualizationSocket.class);
    }

}