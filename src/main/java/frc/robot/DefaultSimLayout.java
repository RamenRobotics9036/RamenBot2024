package frc.robot;

import java.util.AbstractMap;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

/**
 * DefaultSimLayout manages the position and size of various widgets
 * within a user interface. It maintains a mapping of widget names
 * to their respective positions and dimensions.
 * 
 * <p>
 * Each widget is represented by a {@link Widget} object which stores
 * its x and y coordinates, width and height. This mapping is stored
 * in a HashMap.
 * </p>
 *
 * <p>
 * This class is initialized with a set of predefined widget positions,
 * and a public method is provided to retrieve the position of a widget
 * by its name.
 * </p>
 *
 * <p>
 * Example usage:
 * 
 * <pre>
 * DefaultSimLayout layout = new DefaultSimLayout();
 * Widget widgetPosition = layout.getWidgetPosition("Field");
 * System.out.println("X: " + widgetPosition.getX());
 * </pre>
 * </p>
 */
public class DefaultSimLayout {
    /**
     * Widget position and dimensions.
     */
    public static class Widget {
        @SuppressWarnings("MemberNameCheck")
        public int x;
        @SuppressWarnings("MemberNameCheck")
        public int y;
        @SuppressWarnings("MemberNameCheck")
        public int width;
        @SuppressWarnings("MemberNameCheck")
        public int height;

        /**
         * Constructor.
         */
        public Widget(int xpos, int ypos, int width, int height) {
            this.x = xpos;
            this.y = ypos;
            this.width = width;
            this.height = height;
        }
    }

    private static final Map<String, Widget> widgetMap;

    static {
        List<AbstractMap.SimpleEntry<String, Widget>> widgets = Arrays.asList(
                new AbstractMap.SimpleEntry<>("Field", new Widget(2, 0, 5, 3)),
                new AbstractMap.SimpleEntry<>("Heading", new Widget(0, 0, 2, 2)));

        widgetMap = widgets.stream().collect(
                Collectors
                        .toMap(AbstractMap.SimpleEntry::getKey, AbstractMap.SimpleEntry::getValue));
    }

    /**
     * Returns the position of a widget by its name.
     */
    public Widget getWidgetPosition(String widgetName) {
        Widget result = widgetMap.get(widgetName);

        if (result == null) {
            throw new IllegalArgumentException("Widget layout not found: " + widgetName);
        }

        return result;
    }
}
